from setuptools import setup, find_packages

setup(
    name="artefacts-client",
    packages=find_packages(),
    install_requires=[
        "requests==2.27.1",
        "click>=8.1",
        "junitparser>=2.5",
        "PyYAML>=6.0",
    ],
    include_package_data=True,
    entry_points={
        "console_scripts": [
            "warpcli = artefacts.scripts.warpcli:warpcli",
        ],
    },
)
