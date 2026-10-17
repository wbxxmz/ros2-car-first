from setuptools import setup

package_name = "ican_inspection_examples"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_data={package_name: ["templates/*", "models/*"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="iCAN Sim",
    maintainer_email="todo@example.com",
    description="Minimal user templates for the iCAN task 1 inspection simulator.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "minimal_user_node = ican_inspection_examples.minimal_user_node:main",
        ],
    },
)
