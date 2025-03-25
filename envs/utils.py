import xml.etree.ElementTree as ET

def get_object_body(
        obj_type='box', 
        size=[0.01, 0.01, 0.1], 
        density=1000,
        *args,
        **kwargs
        ):
    if obj_type == 'box':
        new_body = ET.Element("body", {"name": "object", "pos": f"0.0 0.0 {size[2]*0.5 + 0.005}"})
        freejoint = ET.Element("freejoint")
        new_body.append(freejoint)
        box_geom = ET.Element("geom", {
            "name": "object",
            "type": "box",
            "size": f"{size[0]} {size[1]} {size[2]}",
            "pos": "0 0 0",
            "rgba": "0.7 0.2 0.1 0.6",
            "density": f"{density}",
            "class": "contact",
        })
        new_body.append(box_geom)
    elif obj_type == 'cylinder':
        new_body = ET.Element("body", {"name": "object", "pos": f"0.0 0.0 {size[1]*0.5 + 0.005}"})
        freejoint = ET.Element("freejoint")
        new_body.append(freejoint)
        cylinder_geom = ET.Element("geom", {
            "name": "object",
            "type": "cylinder",
            "size": f"{size[0]} {size[1]}",
            "pos": "0 0 0",
            "rgba": "0.7 0.2 0.1 0.6",
            "density": f"{density}",
            "class": "contact",
        })
        new_body.append(cylinder_geom)
    return new_body