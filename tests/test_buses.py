"""A repeated part's buses are split per chain; every other body's ports are its bindings'."""

from sx_embodiments import development_embodiments, embodiments
from sx_embodiments.buses import driven_axes


def _bodies():
    for registry in (embodiments, development_embodiments):
        for name in registry:
            yield registry[name]


def test_a_part_repeated_on_one_bus_kind_gets_a_bus_per_chain() -> None:
    axes = driven_axes(embodiments["bimanual-so101"])
    assert [axis.port for axis in axes] == ["left_arm_serial"] * 6 + ["right_arm_serial"] * 6
    assert axes[0].joint == "left_shoulder_pan" and axes[6].joint == "right_shoulder_pan"
    assert len({axis.joint for axis in axes}) == len(axes)


def test_a_body_that_reuses_no_address_keeps_its_bindings_ports_and_joint_names() -> None:
    for body in _bodies():
        axes = driven_axes(body)
        if any(axis.port != axis.binding.bus_port for axis in axes):
            continue
        names = [c.joint_name for c in body.state.coordinates if c.axis.actuator is not None]
        if len(set(names)) == len(names):  # a part repeated with its own names is renamed
            assert [axis.joint for axis in axes] == names, body.name


def test_every_driven_bus_address_is_unique_once_split() -> None:
    for body in _bodies():
        addresses = [(a.binding.bus, a.port, a.binding.bus_id) for a in driven_axes(body)]
        assert len(set(addresses)) == len(addresses), body.name
