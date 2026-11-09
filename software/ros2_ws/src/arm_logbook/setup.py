from setuptools import find_packages, setup

package_name = "arm_logbook"

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
    description="Logbook of the arm in SQLite: last position, calibrations, recordings, faults.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["logbook_node = arm_logbook.node:main", "logbook = arm_logbook.cli:main"]},
)
