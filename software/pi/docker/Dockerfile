# Περιβάλλον του Raspberry Pi: ROS 2 Jazzy (χωρίς γραφικά) + I²C, για το arm_pi_driver.
# Ο κώδικας ΔΕΝ μπαίνει στο image: το compose.yaml «συνδέει» τον φάκελο του repo.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-smbus2 \
        i2c-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/master-s-thesis