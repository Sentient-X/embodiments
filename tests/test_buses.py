"""A repeated part's buses are split per chain; every other body's ports are its bindings'."""

from sx_embodiments import development_embodiments, embodiments
from sx_embodiments.buses import driven_axes

#: The bodies that repeat a part with its addresses on one bus kind, which alone are split.
SPLIT = {"bimanual-so101", "bimanual-b601-dm", "b601-dm-station"}


def _bodies():
    for registry in (embodiments, development_embodiments):
        for name in registry:
            yield registry[name]


def test_a_part_repeated_on_one_bus_kind_gets_a_bus_per_chain() -> None:
    axes = driven_axes(embodiments["bimanual-so101"])
    assert [axis.port for axis in axes] == ["left_arm_serial"] * 6 + ["right_arm_serial"] * 6
    assert axes[0].joint == "left_shoulder_pan" and axes[6].joint == "right_shoulder_pan"
    assert len({axis.joint for axis in axes}) == len(axes)


def test_only_bodies_that_repeat_an_address_are_split_and_the_rest_read_as_bound() -> None:
    split: set[str] = set()
    for body in _bodies():
        axes = driven_axes(body)
        coordinates = [c for c in body.state.coordinates if c.axis.actuator is not None]
        if any(axis.port != axis.binding.bus_port for axis in axes):
            split.add(str(body.name))
            continue
        assert [axis.port for axis in axes] == [axis.binding.bus_port for axis in axes]
        assert [axis.joint for axis in axes] == [c.joint_name for c in coordinates], body.name
    assert split == SPLIT


def test_every_driven_bus_address_is_unique_once_split() -> None:
    for body in _bodies():
        addresses = [(a.binding.bus, a.port, a.binding.bus_id) for a in driven_axes(body)]
        assert len(set(addresses)) == len(addresses), body.name
