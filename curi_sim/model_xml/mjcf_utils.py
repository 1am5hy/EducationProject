import xml.etree.ElementTree as ET
import os
import numpy as np
from collections.abc import Iterable
from PIL import Image
from pathlib import Path
from copy import deepcopy

SENSOR_TYPES = {
    "touch",
    "accelerometer",
    "velocimeter",
    "gyro",
    "force",
    "torque",
    "magnetometer",
    "rangefinder",
    "jointpos",
    "jointvel",
    "tendonpos",
    "tendonvel",
    "actuatorpos",
    "actuatorvel",
    "actuatorfrc",
    "ballangvel",
    "jointlimitpos",
    "jointlimitvel",
    "jointlimitfrc",
    "tendonlimitpos",
    "tendonlimitvel",
    "tendonlimitfrc",
    "framepos",
    "framequat",
    "framexaxis",
    "frameyaxis",
    "framezaxis",
    "framelinvel",
    "frameangvel",
    "framelinacc",
    "frameangacc",
    "subtreecom",
    "subtreelinvel",
    "subtreeangmom",
    "user",
}


def find_elements(root, tags, attribs=None, return_first=True):
    """
    Find all element(s) matching the requested @tag and @attributes. If @return_first is True, then will return the
    first element found matching the criteria specified. Otherwise, will return a list of elements that match the
    criteria.
    Args:
        root (ET.Element): Root of the xml element tree to start recursively searching through.
        tags (str or list of str or set): Tag(s) to search for in this ElementTree.
        attribs (None or dict of str): Element attribute(s) to check against for a filtered element. A match is
            considered found only if all attributes match. Each attribute key should have a corresponding value with
            which to compare against.
        return_first (bool): Whether to immediately return once the first matching element is found.
    Returns:
        None or ET.Element or list of ET.Element: Matching element(s) found. Returns None if there was no match.
    """
    # Initialize return value
    elements = None if return_first else []

    # Make sure tags is list
    tags = [tags] if type(tags) is str else tags

    # Check the current element for matching conditions
    if root.tag in tags:
        matching = True
        if attribs is not None:
            for k, v in attribs.items():
                if root.get(k) != v:
                    matching = False
                    break
        # If all criteria were matched, add this to the solution (or return immediately if specified)
        if matching:
            if return_first:
                return root
            else:
                elements.append(root)
    # Continue recursively searching through the element tree
    for r in root:
        if return_first:
            elements = find_elements(tags=tags, attribs=attribs, root=r, return_first=return_first)
            if elements is not None:
                return elements
        else:
            found_elements = find_elements(tags=tags, attribs=attribs, root=r, return_first=return_first)
            pre_elements = deepcopy(elements)
            if found_elements:
                elements += found_elements if type(found_elements) is list else [found_elements]

    return elements if elements else None

def _element_filter(element, parent):
    """
    Default element filter to be used in sort_elements. This will filter for the following groups:
        :`'root_body'`: Top-level body element
        :`'bodies'`: Any body elements
        :`'joints'`: Any joint elements
        :`'actuators'`: Any actuator elements
        :`'sites'`: Any site elements
        :`'sensors'`: Any sensor elements
        :`'contact_geoms'`: Any geoms used for collision (as specified by group 0 (default group) geoms)
        :`'visual_geoms'`: Any geoms used for visual rendering (as specified by group 1 geoms)
    Args:
        element (ET.Element): Current XML element that we are filtering
        parent (ET.Element): Parent XML element for the current element
    Returns:
        str or None: Assigned filter key for this element. None if no matching filter is found.
    """
    # Check for actuator first since this is dependent on the parent element
    if parent is not None and parent.tag == "actuator":
        return "actuators"
    elif element.tag == "joint":
        # Make sure this is not a tendon (this should not have a "joint", "joint1", or "joint2" attribute specified)
        if element.get("joint") is None and element.get("joint1") is None:
            return "joints"
    elif element.tag == "body":
        # If the parent of this does not have a tag "body", then this is the top-level body element
        if parent is None or parent.tag != "body":
            return "root_body"
        return "bodies"
    elif element.tag == "site":
        return "sites"
    elif element.tag in SENSOR_TYPES:
        return "sensors"
    elif element.tag == "geom":
        # Only get collision and visual geoms (group 0 / None, or 1, respectively)
        group = element.get("group")
        if group in {None, "0", "1"}:
            return "visual_geoms" if group == "1" else "contact_geoms"
    else:
        # If no condition met, return None
        return None


def sort_elements(root, parent=None, element_filter=None, _elements_dict=None):
    """
    Utility method to iteratively sort all elements based on @tags. This XML ElementTree will be parsed such that
    all elements with the same key as returned by @element_filter will be grouped as a list entry in the returned
    dictionary.
    Args:
        root (ET.Element): Root of the xml element tree to start recursively searching through
        parent (ET.Element): Parent of the root node. Default is None (no parent node initially)
        element_filter (None or function): Function used to filter the incoming elements. Should take in two
            ET.Elements (current_element, parent_element) and return a string filter_key if the element
            should be added to the list of values sorted by filter_key, and return None if no value should be added.
            If no element_filter is specified, defaults to self._element_filter.
        _elements_dict (dict): Dictionary that gets passed to recursive calls. Should not be modified externally by
            top-level call.
    Returns:
        dict: Filtered key-specific lists of the corresponding elements
    """
    # Initialize dictionary and element filter if None is set
    if _elements_dict is None:
        _elements_dict = {}
    if element_filter is None:
        element_filter = _element_filter

    # Parse this element
    key = element_filter(root, parent)
    if key is not None:
        # Initialize new entry in the dict if this is the first time encountering this value, otherwise append
        if key not in _elements_dict:
            _elements_dict[key] = [root]
        else:
            _elements_dict[key].append(root)

    # Loop through all possible subtrees for this XML recurisvely
    for r in root:
        _elements_dict = sort_elements(
            root=r,
            parent=root,
            element_filter=element_filter,
            _elements_dict=_elements_dict
        )

    return _elements_dict


