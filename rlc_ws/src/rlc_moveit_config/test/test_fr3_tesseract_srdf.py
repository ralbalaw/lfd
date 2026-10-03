# Copyright 2026 HRI-RLC Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Verify that the FR3 Tesseract SRDF preserves arm and collision semantics."""

from pathlib import Path
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
import xacro


def _render_xacro(path: Path, *, hand: bool) -> ElementTree.Element:
    """
    Render an FR3 semantic Xacro file.

    Parameters
    ----------
    path : pathlib.Path
        Semantic Xacro file to render.
    hand : bool
        Whether the Franka hand is included in the semantic configuration.

    Returns
    -------
    xml.etree.ElementTree.Element
        Rendered SRDF root element.

    """
    document = xacro.process_file(
        str(path),
        mappings={'hand': str(hand).lower(), 'ee_id': 'franka_hand'},
    )
    return ElementTree.fromstring(document.toxml())


def _render_urdf(path: Path, *, hand: bool) -> ElementTree.Element:
    """
    Render the FR3 robot-description Xacro file.

    Parameters
    ----------
    path : pathlib.Path
        Robot-description Xacro file to render.
    hand : bool
        Whether the Franka hand is included in the robot model.

    Returns
    -------
    xml.etree.ElementTree.Element
        Rendered URDF root element.

    """
    document = xacro.process_file(
        str(path),
        mappings={
            'ros2_control': 'false',
            'hand': str(hand).lower(),
            'ee_id': 'franka_hand',
            'robot_type': 'fr3',
        },
    )
    return ElementTree.fromstring(document.toxml())


def _named_elements(
    root: ElementTree.Element, tag: str
) -> dict[str, ElementTree.Element]:
    """
    Index direct SRDF children by their `name` attribute.

    Parameters
    ----------
    root : xml.etree.ElementTree.Element
        Rendered SRDF root element.
    tag : str
        Direct-child XML tag to index.

    Returns
    -------
    dict[str, xml.etree.ElementTree.Element]
        Elements keyed by name.

    """
    return {element.attrib['name']: element for element in root.findall(tag)}


def _group_states(
    root: ElementTree.Element, group: str
) -> dict[str, dict[str, str]]:
    """
    Extract named joint states for one semantic group.

    Parameters
    ----------
    root : xml.etree.ElementTree.Element
        Rendered SRDF root element.
    group : str
        Semantic group name.

    Returns
    -------
    dict[str, dict[str, str]]
        State names mapped to joint-value strings.

    """
    return {
        state.attrib['name']: {
            joint.attrib['name']: joint.attrib['value']
            for joint in state.findall('joint')
        }
        for state in root.findall('group_state')
        if state.attrib['group'] == group
    }


def _collision_pairs(root: ElementTree.Element) -> set[tuple[str, str]]:
    """
    Extract disabled-collision entries as unordered link pairs.

    Parameters
    ----------
    root : xml.etree.ElementTree.Element
        Rendered SRDF root element.

    Returns
    -------
    set[tuple[str, str]]
        Canonically ordered link-name pairs.

    """
    return {
        tuple(sorted((entry.attrib['link1'], entry.attrib['link2'])))
        for entry in root.findall('disable_collisions')
    }


def _arm_chain(root: ElementTree.Element) -> dict[str, str]:
    """
    Extract the single chain declaration for `fr3_arm`.

    Parameters
    ----------
    root : xml.etree.ElementTree.Element
        Rendered SRDF root element.

    Returns
    -------
    dict[str, str]
        Chain attributes containing `base_link` and `tip_link`.

    """
    arm = _named_elements(root, 'group')['fr3_arm']
    chain = arm.find('chain')
    assert chain is not None
    return chain.attrib


def _chain_joint_names(
    urdf: ElementTree.Element, *, base_link: str, tip_link: str
) -> list[str]:
    """
    Extract movable joint names along one URDF link chain.

    Parameters
    ----------
    urdf : xml.etree.ElementTree.Element
        Rendered URDF root element.
    base_link : str
        Link at which traversal stops, excluded from the result.
    tip_link : str
        Link from which traversal starts.

    Returns
    -------
    list[str]
        Movable joint names ordered from base to tip.

    """
    joints_by_child = {
        joint.find('child').attrib['link']: joint for joint in urdf.findall('joint')
    }
    names = []
    link = tip_link
    while link != base_link:
        joint = joints_by_child[link]
        if joint.attrib['type'] != 'fixed':
            names.append(joint.attrib['name'])
        link = joint.find('parent').attrib['link']
    names.reverse()
    return names


def _contact_manager_config(root: ElementTree.Element) -> str:
    """
    Extract the Tesseract contact-manager plugin configuration URI.

    Parameters
    ----------
    root : xml.etree.ElementTree.Element
        Rendered SRDF root element.

    Returns
    -------
    str
        Package URI of the contact-manager plugin configuration.

    """
    config = root.find('contact_managers_plugin_config')
    assert config is not None
    return config.attrib['filename']


def test_tesseract_srdf_preserves_loaded_hand_collision_semantics() -> None:
    """Preserve arm semantics and all collision pairs while omitting the hand group."""
    package_root = Path(__file__).parents[1]
    franka_root = Path(get_package_share_directory('franka_description'))
    moveit_srdf = _render_xacro(franka_root / 'robots/fr3/fr3.srdf.xacro', hand=True)
    urdf = _render_urdf(franka_root / 'robots/fr3/fr3.urdf.xacro', hand=True)
    tesseract_srdf = _render_xacro(
        package_root / 'config/fr3/fr3_tesseract.srdf.xacro', hand=True
    )

    assert _arm_chain(tesseract_srdf) == _arm_chain(moveit_srdf)
    assert _arm_chain(tesseract_srdf) == {
        'base_link': 'fr3_link0',
        'tip_link': 'fr3_hand_tcp',
    }
    assert _group_states(tesseract_srdf, 'fr3_arm') == _group_states(
        moveit_srdf, 'fr3_arm'
    )
    assert _collision_pairs(tesseract_srdf) == _collision_pairs(moveit_srdf)
    assert _contact_manager_config(tesseract_srdf) == (
        'package://tesseract_support/urdf/contact_manager_plugins.yaml'
    )
    assert _chain_joint_names(
        urdf, base_link='fr3_link0', tip_link='fr3_hand_tcp'
    ) == [f'fr3_joint{index}' for index in range(1, 8)]
    assert {'fr3_hand', 'fr3_hand_tcp', 'fr3_leftfinger', 'fr3_rightfinger'} <= {
        link.attrib['name'] for link in urdf.findall('link')
    }

    groups = _named_elements(tesseract_srdf, 'group')
    assert set(groups) == {'fr3_arm'}
    assert _group_states(tesseract_srdf, 'fr3_hand') == {}
    assert tesseract_srdf.findall('end_effector') == []


def test_tesseract_srdf_preserves_arm_only_semantics() -> None:
    """Match Franka arm semantics when the hand is disabled."""
    package_root = Path(__file__).parents[1]
    franka_root = Path(get_package_share_directory('franka_description'))
    moveit_srdf = _render_xacro(franka_root / 'robots/fr3/fr3.srdf.xacro', hand=False)
    tesseract_srdf = _render_xacro(
        package_root / 'config/fr3/fr3_tesseract.srdf.xacro', hand=False
    )

    assert _arm_chain(tesseract_srdf) == _arm_chain(moveit_srdf)
    assert _arm_chain(tesseract_srdf)['tip_link'] == 'fr3_link8'
    assert _group_states(tesseract_srdf, 'fr3_arm') == _group_states(
        moveit_srdf, 'fr3_arm'
    )
    assert _collision_pairs(tesseract_srdf) == _collision_pairs(moveit_srdf)
    assert _contact_manager_config(tesseract_srdf) == (
        'package://tesseract_support/urdf/contact_manager_plugins.yaml'
    )
    assert set(_named_elements(tesseract_srdf, 'group')) == {'fr3_arm'}
