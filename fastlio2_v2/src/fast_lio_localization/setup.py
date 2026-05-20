from setuptools import setup
import os
import glob

package_name = "fast_lio_localization"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Fast LIO Localization - global relocalization in pre-built maps",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "global_localization = fast_lio_localization.global_localization:main",
            "publish_initial_pose = fast_lio_localization.publish_initial_pose:main",
            "transform_fusion = fast_lio_localization.transform_fusion:main",
            "invert_livox_scan = fast_lio_localization.invert_livox_scan:main",
        ],
    },
    data_files=[
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob.glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob.glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "maps"), glob.glob("maps/*")),
    ],
)
