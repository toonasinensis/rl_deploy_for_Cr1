import xml.etree.ElementTree as ET

def parse_urdf(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    print(f"{'Joint Name':<25} {'Pos Limit (lower, upper)':<25} {'Torque Limit'}")
    print("-" * 70)

    for joint in root.findall('joint'):
        name = joint.get('name')
        joint_type = joint.get('type')

        # Only care about revolute or prismatic joints
        if joint_type not in ['revolute', 'prismatic']:
            continue

        limit_tag = joint.find('limit')
        if limit_tag is not None:
            lower = limit_tag.get('lower', 'None')
            upper = limit_tag.get('upper', 'None')
            effort = limit_tag.get('effort', 'None')
        else:
            lower, upper, effort = 'None', 'None', 'None'

        print(f"{name:<25} ({lower}, {upper}){'':<5} {effort}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Read URDF and print joint limits")
    parser.add_argument("urdf_path", help="Path to URDF file")
    args = parser.parse_args()

    parse_urdf(args.urdf_path)



