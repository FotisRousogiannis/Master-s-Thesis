"""Ο server ηχούς: τρέχει στο Pi και επιστρέφει κάθε πακέτο σε όποιον το έστειλε."""

from arm_link.transport import Transport, UdpTransport

DEFAULT_PORT = 9999


class EchoServer:
    def __init__(self, transport: Transport):
        self.transport = transport

    def serve(self) -> None:
        while True:
            data = self.transport.receive(1.0)
            if data is not None:
                self.transport.send(data)


if __name__ == "__main__":
    transport = UdpTransport(local_port=DEFAULT_PORT)
    print(f"EchoServer ακούει στη θύρα {DEFAULT_PORT} (Ctrl+C για τέλος)")
    try:
        EchoServer(transport).serve()
    except KeyboardInterrupt:
        pass
    finally:
        transport.close()