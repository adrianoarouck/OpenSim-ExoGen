# legacy/

Arquivos do fluxo anterior ao `exo_builder`, guardados para referência. Nada
aqui é usado pelo pacote, pelos testes ou pelos exemplos.

| Pasta | Conteúdo |
|---|---|
| `scripts/` | `exo_generator.py` (gerador original), `aid_functions.py`, `IK_setup_virtual_marker.py` (marcadores virtuais no .trc), `exampleMocoTrack.py`, `osimversiontest.py` |
| `models/` | modelos antigos: `subject3.osim`, `test-s11.osim`, `human-with-exo.osim`, `subject3_withexo_scaled.osim` (LaiArnold com braços) e as saídas do gerador original `test-output.osim` e `test-output_virtual-markers.osim` |
| `setups/` | setups de IK/ID/Scale (`STStest.xml`, `walk1test-setup.xml`, `walk3test-setup.xml`, `scaletool_*.xml`) e `grf_walk.xml` |
| `outputs/` | resultados antigos: `IK-walking1.mot`, `walk3test.mot`, `inverse_dynamics.sto` |

Observações:

* Os scripts usam caminhos relativos da estrutura antiga (ex.: `osim.Model("subject_walk_contact.osim")`,
  `Walking-tests\...`). Para rodá-los, ajuste para `../../models/subject_walk_contact.osim` e
  `../../data/walking/...`.
* Vários setups apontam para caminhos de outra máquina (`C:/MyDriveSym/...`).
* Os modelos daqui procuram malhas em `legacy/models/Geometry`, que não existe; o
  OpenSim só avisa e mostra o modelo sem ossos. Copie ou aponte para `models/Geometry` se precisar.
* `exampleMocoTrack.py` tem dois bugs conhecidos: remove atuadores com índice fixo
  (`remove(qt)`) e passa o `modelProcessor` original ao `track.setModel`, descartando
  o modelo modificado. Use `examples/moco_track_exo.py`.
* Duas cópias idênticas foram removidas na reorganização: `walking1.trc` (=
  `data/walking/MARKERS/walking1.trc`) e `walk1test` (= `data/walking/walking1subject3/ikresults1`).
