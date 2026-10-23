# legacy/

Files from the workflow that preceded `exo_builder`, kept for reference. Nothing
here is used by the package, the tests or the examples.

| Folder | Contents |
|---|---|
| `scripts/` | `exo_generator.py` (original generator), `aid_functions.py`, `IK_setup_virtual_marker.py` (virtual markers added to the .trc), `exampleMocoTrack.py`, `osimversiontest.py` |
| `models/` | older models: `subject3.osim`, `test-s11.osim`, `human-with-exo.osim`, `subject3_withexo_scaled.osim` (LaiArnold with arms) and the outputs of the original generator, `test-output.osim` and `test-output_virtual-markers.osim` |
| `setups/` | IK/ID/Scale setups (`STStest.xml`, `walk1test-setup.xml`, `walk3test-setup.xml`, `scaletool_*.xml`) and `grf_walk.xml` |
| `outputs/` | old results: `IK-walking1.mot`, `walk3test.mot`, `inverse_dynamics.sto` |

Notes:

* The scripts use relative paths from the old layout (e.g.
  `osim.Model("subject_walk_contact.osim")`, `Walking-tests\...`). To run them,
  change these to `../../models/subject_walk_contact.osim` and `../../data/walking/...`.
* Several setups point to paths on another machine (`C:/MyDriveSym/...`).
* The models here look for meshes in `legacy/models/Geometry`, which does not exist;
  OpenSim only warns and shows the model without bones. Copy or point to
  `models/Geometry` if you need them.
* `exampleMocoTrack.py` has two known bugs: it removes actuators with a fixed index
  (`remove(qt)`) and passes the original `modelProcessor` to `track.setModel`,
  discarding the modified model. Use `examples/moco_track_exo.py` instead.
* Two identical copies were removed during the reorganisation: `walking1.trc`
  (= `data/walking/MARKERS/walking1.trc`) and `walk1test`
  (= `data/walking/walking1subject3/ikresults1`).
