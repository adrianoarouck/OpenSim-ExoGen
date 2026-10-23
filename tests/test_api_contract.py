"""OpenSim Python API behaviour exo_builder relies on (verified on 4.5 and 4.6).

If one of these fails after an OpenSim upgrade, fix the matching helper before
trusting any result.
"""
import opensim as osim
import numpy as np
import pytest

V = osim.Vec3


def approx(v):
    return pytest.approx(v, abs=1e-6)


def two_body_model():
    m = osim.Model()
    m.setGravity(V(0))
    a = osim.Body("a", 1, V(0), osim.Inertia(.1))
    b = osim.Body("b", 1, V(0), osim.Inertia(.1))
    m.addBody(a)
    m.addBody(b)
    m.addJoint(osim.WeldJoint("wa", m.getGround(), a))
    jb = osim.FreeJoint("jb", m.getGround(), b)
    m.addJoint(jb)
    fa = osim.PhysicalOffsetFrame("fa", a, osim.Transform(V(0.5, 0, 0)))
    fb = osim.PhysicalOffsetFrame("fb", b, osim.Transform(V(0.5, 0, 0)))
    a.addComponent(fa)
    b.addComponent(fb)
    return m, a, b, fa, fb, jb


def test_bushing_record_layout_and_torque_reference():
    m, a, b, fa, fb, jb = two_body_model()
    bf = osim.BushingForce("bush", fa, fb, V(1000, 1000, 1000), V(0, 0, 0), V(0, 0, 0), V(0, 0, 0))
    m.addForce(bf)
    s = m.initSystem()
    jb.upd_coordinates(4).setValue(s, 0.01)  # FreeJoint: rx, ry, rz, tx, ty, tz -> index 4 = ty
    m.realizeVelocity(s)
    rec = bf.getRecordValues(s)
    vals = [rec.get(i) for i in range(12)]
    assert vals[1] == approx(10.0)       # force on frame1 body, ground y
    assert vals[5] == approx(5.0)        # torque about BODY ORIGIN: 0.5 m x 10 N
    assert vals[7] == approx(-10.0)      # reaction on frame2 body


def test_expression_bushing_variables_and_sign():
    m, a, b, fa, fb, jb = two_body_model()
    f = osim.ExpressionBasedBushingForce("eb", fa, fb)
    f.setFxExpression("1000*delta_x + 5e5*delta_x^3")
    m.addForce(f)
    s = m.initSystem()
    jb.upd_coordinates(3).setValue(s, 0.01)
    m.realizeVelocity(s)
    assert f.getRecordValues(s).get(0) == approx(10.5)  # restoring, like BushingForce


def test_duplicate_coordinate_names_are_silently_accepted():
    # This is why ExoBuilder checks name uniqueness itself.
    m = osim.Model()
    for n in ("b1", "b2"):
        b = osim.Body(n, 1, V(0), osim.Inertia(.1))
        m.addBody(b)
        j = osim.PinJoint(f"j{n}", m.getGround(), b)
        j.updCoordinate().setName("q")
        m.addJoint(j)
    m.finalizeConnections()
    m.initSystem()


def test_add_component_transfers_ownership():
    b = osim.Body("b", 1, V(0), osim.Inertia(1))
    f = osim.PhysicalOffsetFrame()
    f.setName("f")
    f.setParentFrame(b)
    b.addComponent(f)
    assert not f.thisown


def test_free_joint_speeds_are_not_qdot():
    m = osim.Model()
    b = osim.Body("b", 1, V(0), osim.Inertia(.1))
    m.addBody(b)
    m.addJoint(osim.FreeJoint("j", m.getGround(), b))
    s = m.initSystem()
    m.getCoordinateSet().get(0).setValue(s, 0.8)  # rx
    m.realizePosition(s)
    qd = osim.Vector(s.getNQ(), 0.0)
    qd.set(1, 1.0)  # d(ry)/dt
    u = osim.Vector(s.getNU(), 0.0)
    m.getMatterSubsystem().multiplyByNInv(s, False, qd, u)
    assert abs(u.get(1) - 1.0) > 0.1 or abs(u.get(2)) > 0.1  # u != qdot when rx != 0


def test_transform_axis_coordinate_names_accessor():
    ta = osim.TransformAxis()
    assert hasattr(ta, "getCoordinateNamesInArray")


def test_euler_convention_matches_offset_frames():
    from exo_builder.geometry import offset_frame, rot_from_xyz, transform_np
    ang = np.array([0.3, -0.7, 1.1])
    m = osim.Model()
    b = osim.Body("b", 1, V(0), osim.Inertia(.1))
    m.addBody(b)
    m.addJoint(osim.WeldJoint("w", m.getGround(), b))
    f = offset_frame("f", b, [0.1, 0.2, 0.3], rot_from_xyz(ang))
    b.addComponent(f)
    s = m.initSystem()
    R, p = transform_np(f, s)
    assert np.allclose(R, rot_from_xyz(ang)) and np.allclose(p, [0.1, 0.2, 0.3])
