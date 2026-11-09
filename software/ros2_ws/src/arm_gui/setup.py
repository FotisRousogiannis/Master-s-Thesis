from setuptools import find_packages, setup

package_name = "arm_gui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Fotis Rousogiannis",
    maintainer_email="115790877+FotisRousogiannis@users.noreply.github.com",
    description="Arm Studio: control application for the robotic arm.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["arm_gui = arm_gui.main:main"]},
)
