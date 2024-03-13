from glob import glob

import xml.etree.ElementTree as ET

from setuptools import find_packages, setup

package_name = 'ur_web_gui'
package_xml = ET.parse('package.xml').getroot()

setup(
    name=package_name,
    version=package_xml.find('version').text,
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer=package_xml.find('maintainer').text,
    maintainer_email=package_xml.find('maintainer').attrib['email'],
    description=package_xml.find('description').text,
    license=package_xml.find('license').text,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur_web_gui = ur_web_gui.gui_node:main'
        ],
    },
)
