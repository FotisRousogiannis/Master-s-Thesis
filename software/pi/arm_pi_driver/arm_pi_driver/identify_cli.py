"""Η αναγνώριση από τερματικό, χωρίς Arm Studio (δοκιμή στον πάγκο).

    ros2 run arm_pi_driver identify scan
    ros2 run arm_pi_driver identify read
    ros2 run arm_pi_driver identify jog servo_1=0.4 servo_2=-0.4 [--duration 0.5] [--stop 3]
    ros2 run arm_pi_driver identify exit

Στέλνει ένα αίτημα στο /arm_pi_driver/identify/request και τυπώνει την απάντηση (για jog: και το
αποτέλεσμα). Ο driver μένει σε αναγνώριση 5 s μετά το τελευταίο αίτημα και μετά γυρίζει μόνος του
στον κανονικό έλεγχο.
"""

import argparse
import json
import time

REQUEST, REPLY = '/arm_pi_driver/identify/request', '/arm_pi_driver/identify/reply'


def parse(argv=None) -> dict:
    """Οι παράμετροι της γραμμής εντολών → το αίτημα (JSON)."""
    parser = argparse.ArgumentParser(prog='identify', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('op', choices=['scan', 'read', 'jog', 'stop', 'exit'])
    parser.add_argument('servos', nargs='*', help='για jog: όνομα=ταχύτητα, π.χ. servo_1=0.4')
    parser.add_argument('--duration', type=float, default=0.5, help='μέγιστη διάρκεια [s]')
    parser.add_argument('--stop', type=float, default=3.0, help='στάση όταν ένας encoder '
                                                                'κινηθεί τόσο [°]')
    args = parser.parse_args(argv)
    request = {'id': int(time.time() * 1000) % 1_000_000, 'op': args.op}
    if args.op == 'jog':
        try:
            request['servos'] = {name: float(v) for name, v in
                                 (item.split('=', 1) for item in args.servos)}
        except ValueError:
            parser.error('servo ως όνομα=ταχύτητα, π.χ. servo_1=0.4')
        request.update(duration_s=args.duration, stop_deg=args.stop)
    return request


def main(argv=None):
    """Ένα αίτημα, οι απαντήσεις του, έξοδος."""
    import rclpy                    # εδώ: το parse() δοκιμάζεται χωρίς ROS
    from rclpy.node import Node
    from std_msgs.msg import String

    request = parse(argv)
    rclpy.init()
    node = Node('arm_identify_cli')
    replies = []
    node.create_subscription(String, REPLY, lambda m: replies.append(json.loads(m.data)), 10)
    publisher = node.create_publisher(String, REQUEST, 10)
    deadline = time.monotonic() + 2.0
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)       # περιμένει να «δει» τον driver
    publisher.publish(String(data=json.dumps(request)))
    expected = 2 if request['op'] == 'jog' else 1    # jog: «ξεκίνησε» + το αποτέλεσμα
    deadline = time.monotonic() + 6.0
    mine = []
    while len(mine) < expected and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        mine = [r for r in replies if r.get('id') == request['id']]
        if mine and not mine[0].get('ok'):
            break
    for reply in mine:
        print(json.dumps(reply, ensure_ascii=False, indent=2))
    if not mine:
        print('καμία απάντηση: τρέχει το arm_pi_driver; (ίδιο ROS_DOMAIN_ID;)')
    node.destroy_node()
    rclpy.try_shutdown()
