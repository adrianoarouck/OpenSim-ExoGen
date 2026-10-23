# OpenSim-ExoGen

Opensim model generator of human body with exoskeleton. Focused on parameters of the exoskeleton.

*Read in English: [README.md](README.md)*

## exo_builder — exoesqueleto configurável para modelos OpenSim

Adiciona um exoesqueleto (atuação direta na junta) em volta de um modelo humano
OpenSim a partir de um arquivo YAML, modela a interface humano–exo (cuffs, cinto,
amarração do pé, contato) e calcula as forças de interação.

## Estrutura

```
exo_builder/        pacote (config, builder, interação, análise, cinemática, CLI)
  presets/          exos prontos em YAML
  viewer/           visualizador web
examples/           moco_track_exo.py (MocoTrack com exo)
tests/              pytest
models/             subject_walk_contact.osim (modelo humano base) + Geometry/ (malhas .vtp)
data/walking/       marcha: MARKERS (.trc), MOCAP-IK (.mot), FORCE-MOT, SETUP-ID, SO-results
legacy/             scripts, modelos, setups e resultados do fluxo antigo (ver legacy/README.md)
```

Os comandos abaixo devem ser rodados a partir da raiz do projeto.

Ambiente alvo: conda `opensim46` (OpenSim 4.6, Python 3.11). Dependências extras:
`pip install pyyaml pytest` (numpy, scipy e matplotlib já existem no ambiente).

> **Regra de ouro:** a primeira importação de qualquer script deve ser
> `import exo_builder`. No Windows com OpenSim 4.6 ele resolve três conflitos de
> DLL que derrubam o processo ou o Moco (ver [Problemas de ambiente](#problemas-de-ambiente)).

## Uso rápido

```bash
python -m exo_builder presets
python -m exo_builder build hip_bilateral --model models/subject_walk_contact.osim -o subject_hip_exo.osim
python -m exo_builder fit subject_hip_exo.osim data/walking/MOCAP-IK/walking1.mot -o walking1_exo.mot
python -m exo_builder forces subject_hip_exo.osim walking1_exo.mot -o walking1_cuffs.sto --plot cuffs.png
python -m exo_builder.viewer --model models/subject_walk_contact.osim --config full_bilateral
```

Em Python:

```python
import exo_builder                       # primeiro!
import opensim as osim
model, info = exo_builder.build_exo("models/subject_walk_contact.osim", "my_exo.yaml")
model.printToXML("human_with_exo.osim")
print(info["couplings"], info["warnings"])
```

## Descrevendo um exo (YAML)

Todo arquivo é mesclado sobre os padrões de `exo_builder/config.py::DEFAULTS`, então
basta escrever o que muda. O template comentado completo é
[exo_builder/presets/full_bilateral.yaml](exo_builder/presets/full_bilateral.yaml).
Ângulos em graus, SI no resto, `{s}` vira `r`/`l`.

| Preset | O que é |
|---|---|
| `full_bilateral` | cinto + quadril (flexão atuada, adução/rotação passivas) + joelho + tornozelo, dois lados |
| `full_plate_contact` | igual, mas a placa do pé carrega o contato com o chão |
| `hip_bilateral` | cinto + coxas, só flexão de quadril atuada |
| `hip_knee_left` | unilateral esquerdo, quadril + joelho |
| `knee_right` | joelho direito sem cinto (coxa “flutuante” presa só pelos cuffs) |
| `ankle_bilateral` | canela + placa do pé, sem cinto |

Exemplos de edição:

```yaml
name: meu_exo
sides: [r]                                  # unilateral
segments:
  foot: {enabled: false}                    # remove o pé (e a junta do tornozelo)
  thigh:
    mass: 1.2
    cuffs:                                  # quantos cuffs quiser
      - {name: thigh_prox, along_segment: 0.3, width: 0.06}     # 0 = quadril, 1 = joelho; largura em m
      - name: thigh_dist
        along_segment: 0.8
        model: nonlinear
        stiffness_cubic: {anteroposterior: 2.0e6}               # N/m³
joints:
  knee:
    misalignment: [0.0, 0.02, 0.0]          # eixo do exo 2 cm acima do joelho anatômico
    dofs:
      flexion: {max_torque: 60, limits: soft}
overrides:
  l: {segments: {thigh: {mass: 1.4}}}       # parâmetro diferente só no lado esquerdo
```

Regras da cadeia: `pelvis → thigh → shank → foot`. Juntas existem entre segmentos
habilitados consecutivos (quadril, joelho, tornozelo). Um segmento cujo pai está
desabilitado vira **raiz flutuante** (junta 6-DOF ao ground), presa ao humano só
pelos cuffs. DOFs: `enabled` (senão é travado), `actuated` (cria `tau_exo_*`, um
`CoordinateActuator` com `optimal_force = max_torque`), `passive_stiffness/passive_damping`
(mola/amortecedor torsional na junta), `limits: clamp | soft | none`. O eixo padrão (`axis: human`) é o da
coordenada humana correspondente, então ângulo positivo do exo = do humano.

## Interface humano–exo

Cada acoplamento é um par de frames coincidentes na pose default (pré-carga zero),
um no humano (`<nome>_human`) e outro no exo (`<nome>_exo`), ligados por:

* `linear` → `BushingForce` 6-DOF; `nonlinear` → `ExpressionBasedBushingForce`
  (`F = k·d + cubic·d³`, ou expressões próprias em `delta_x..z`/`theta_x..z`);
* `weld` → `WeldConstraint` (cinto ou pé rígidos).

Direções do frame do cuff (iguais para cuffs, `attach` do cinto e `binding` do pé):

| Chave | Eixo | Sentido positivo |
|---|---|---|
| `anteroposterior` | X | para a frente |
| `longitudinal` | Y | ao longo do segmento, em direção à junta proximal (para cima) |
| `mediolateral` | Z | para a direita do corpo |

Parâmetros de cada acoplamento (por direção):

| Chave | Unidade | O que é |
|---|---|---|
| `stiffness_translational` | N/m | rigidez para o exo se deslocar em relação ao membro |
| `stiffness_rotational` | N·m/rad | rigidez para o exo girar em relação ao membro (em torno de cada eixo) |
| `damping_translational` | N·s/m | amortecimento da velocidade relativa |
| `damping_rotational` | N·m·s/rad | amortecimento da velocidade angular relativa |
| `stiffness_cubic` | N/m³ | só `nonlinear`: F = k·d + k3·d³ |

Cuff: `along_segment` (0–1: 0 = junta proximal, 1 = distal), `width` (m), `radius`
(m ou `auto`), `offset` (m, nas três direções). Todas as chaves com unidade e
descrição estão em `PARAM_DOCS` (`exo_builder/config.py`), que também gera os “?”
do visualizador. Arquivos com os nomes antigos (`at`, `ap/axial/ml`,
`translational`, `cubic`, `stiffness/damping` das juntas) são convertidos
automaticamente ao carregar.

Valores padrão anteroposteriores: coxa k = 3283 N/m, b = 367 N·s/m; canela k = 5645 N/m,
b = 418 N·s/m ([doi:10.1007/s11071-022-07876-8](https://doi.org/10.1007/s11071-022-07876-8)).
Os demais (longitudinal, mediolateral, rotacionais, cinto, amarração do pé) são
estimativas de engenharia — ajuste-os.

Pé (`segments.foot.mode`): `weld` (legado), `bushing` (amarração; só o humano toca
o chão) ou `plate_contact` (as esferas de contato do humano passam a empurrar um
half-space no topo da placa, e a placa ganha esferas próprias contra o chão, com
os mesmos parâmetros).

### Forças de interação

Como a força surge: cada acoplamento liga um ponto do membro a um ponto do exo, que
coincidem na pose de referência. Quando o exo se desloca ou gira em relação ao membro,
a mola-amortecedor gera **F = −K·deslocamento − B·velocidade** (e um momento
**M = −Kr·θ − Br·ω**), expressos nas direções do cuff. Ela age no centro do cuff: no
humano num sentido e no exo no sentido oposto (ação e reação).

`analysis.coupling_loads(model, state)` e `coupling_loads_over_motion(model, mot)`
devolvem, para cada acoplamento, o que o exo aplica **no humano**, no frame do cuff:
`F_ap` (anteroposterior), `F_axial` (longitudinal), `F_ml` (mediolateral), `M_ap`,
`M_axial` (torção em torno do membro), `M_ml`, `F_lat` (+ lateral do lado),
`F_radial` (√(F_ap²+F_ml²), perpendicular ao membro) e `F_total`, em N e N·m.

### Cinemática do exo a partir do humano

`kinematics.fit_exo_to_motion` resolve, quadro a quadro, as coordenadas do exo que
minimizam a energia elástica dos acoplamentos com o humano fixo (pose de um exo
sem massa e sem atuação). Gera um `.mot` completo (humano + exo) para ID/SO, guess
do Moco ou análise de forças. Substitui o `legacy/scripts/IK_setup_virtual_marker.py`.

## Moco

[examples/moco_track_exo.py](examples/moco_track_exo.py): constrói o exo, ajusta a
cinemática, roda `MocoTrack` (sem músculos, reservas só nas coordenadas humanas) e
calcula as forças nos cuffs ao longo da solução. Testado: `hip_bilateral`,
0,5–0,8 s, malha 0,05 s → *Optimal Solution Found* em ~60 s.

Observação sobre o `legacy/scripts/exampleMocoTrack.py`: a remoção de atuadores do exo usa
`remove(qt)` com índice fixo (remove o item errado) e depois `track.setModel(modelProcessor)`
usa o processor original, descartando o modelo modificado.

## Visualizador

`python -m exo_builder.viewer --model models/subject_walk_contact.osim --config full_bilateral`
abre `http://127.0.0.1:8765/`: 3D à esquerda; à direita, abas **Exo** (formulário
gerado da config: qualquer chave nova do YAML aparece sozinha; reconstrói ao editar),
**Pose** (sliders das coordenadas humanas, exo ajustado automaticamente),
**Movimento** (carrega um `.mot`, ajusta e anima, com gráfico por acoplamento) e
**Forças** (tabela). Poses prontas na aba Pose: em pé, sentado, agachamento, meio
agachamento e subindo degrau; com “pés no chão” a altura da pelve é ajustada para o
contato mais baixo do pé tocar o chão. Cada campo mostra a unidade e um “?” com a
explicação. As setas de força têm a ponta no centro do cuff (onde a força age),
aparecem por cima da malha, com o valor em N; dá para ver a resultante ou as
componentes nos eixos do cuff, e alternar entre a força no humano e a reação no exo. Salva YAML e exporta `.osim` (construído no modelo original,
com músculos). Three.js vem do jsdelivr (precisa de internet no navegador).

## Testes

```bash
python -m pytest
```

Inclui contratos da API do OpenSim (comportamentos não documentados dos quais o
pacote depende; passam em 4.5 e 4.6), build de todos os presets com pré-carga zero,
respostas físicas (desalinhamento, cuff não-linear, modos do pé) e um teste
dinâmico: com o humano travado, o exo pendurado assenta e a força total nos
acoplamentos iguala o peso do exo (±3%).

## Problemas de ambiente

Verificados no `opensim46` desta máquina (o `import exo_builder` resolve os três):

1. O `opensim` traz `libblas.dll`/`liblapack.dll` próprias; com numpy importado
   antes, o Simbody usa as do MKL e `assemble()` dá *access violation*.
2. `C:\OpenSim 4.5\bin` está no PATH do sistema (e recebeu os DLLs do CoinHSL em
   8/mai). O CasADi do 4.6 carregava o plugin Ipopt do 4.5 → *Plugin 'ipopt' is not
   found* (WIN32 127). Correção: `CASADIPATH`.
3. O MKL do conda usa um `libiomp5md.dll` que é um *shim* do llvm-openmp
   (repassa para o `libomp.dll` daquela pasta do 4.5); o Ipopt/MUMPS do 4.6 precisam
   do OpenMP da Intel → WIN32 182. Correção: `MKL_THREADING_LAYER=SEQUENTIAL`
   antes do `import opensim`.

Limpeza definitiva (opcional, decisão sua): tirar `C:\OpenSim 4.5\bin` do PATH do
sistema. O HSL pode continuar sendo usado no outro projeto apontando para a pasta
dele explicitamente.

## Armadilhas da API do OpenSim (verificadas)

* Adicionar qualquer componente invalida o System → `human.py` tira um snapshot da pose antes.
* `safeDownCast(obj.clone())` deixa ponteiro pendurado → aborta o Python.
* Nomes de coordenada duplicados são aceitos em silêncio (bug `q_rz`/`q_ry` do legado) → o builder verifica.
* `BushingForce.getRecordValues`: força/torque no corpo do frame1 e frame2, no ground, torque em torno da **origem do corpo**.
* `FreeJoint`: velocidade ≠ derivada da coordenada e o Moco não aceita → raízes usam `CustomJoint` 6-DOF.
* Coordenadas de `CustomJoint` só existem depois de `model.addJoint`.

## Limitações / próximos passos

* Joelho do exo é pino; o *walker knee* humano acopla rotação/adução → forças de
  incompatibilidade (~100 N ML na canela na marcha) com o exo completo. É física do
  modelo, mas vale estudar DOFs passivos extras ou rigidezes.
* Rigidez/amortecimento não-sagitais, cinto e amarração são estimativas.
* A pose default do modelo humano penetra o chão; para simular com `plate_contact`
  é preciso partir de uma pose de equilíbrio.
