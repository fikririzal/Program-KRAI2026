# KFS Retrieval Plan Message

README ini menjelaskan format message hasil planner KFS Retrieval, terutama:

- `KfsRetrievalPlan.msg`
- `ForestPlanStep.msg`
- `ForestPathPoint.msg`

Message ini dipakai untuk mengirim rencana eksekusi robot dari planner menuju executor, dummy executor, dan GUI visualizer.

---

## 1. Struktur Utama `KfsRetrievalPlan`

```msg
builtin_interfaces/Time stamp

int32 step_count
float64 total_cost
int32[] path_nodes
int32[] picked_kfs

ForestPathPoint[] visual_path
ForestPlanStep[] steps
```

Contoh output:

```yaml
stamp:
  sec: 1783158342
  nanosec: 491088003
step_count: 13
total_cost: 19.3
path_nodes:
- 1
- 4
- 7
- 8
- 11
picked_kfs:
- 4
- 7
- 9
- 11
```

---

## 2. Field pada `KfsRetrievalPlan`

### `stamp`

```yaml
stamp:
  sec: 1783158342
  nanosec: 491088003
```

Timestamp saat plan dibuat atau dipublish.

Field ini berguna untuk debugging dan memastikan executor menerima plan terbaru.

---

### `step_count`

```yaml
step_count: 13
```

Jumlah total action step yang harus dieksekusi robot.

Pada contoh, robot memiliki 13 step eksekusi.

---

### `total_cost`

```yaml
total_cost: 19.3
```

Total estimasi cost dari seluruh step.

Nilai ini didapat dari penjumlahan semua `cost` pada `steps`.

```text
total_cost = cost step 1 + cost step 2 + ... + cost step N
```

`total_cost` bukan waktu real robot, tetapi nilai estimasi planner untuk membandingkan efisiensi beberapa kemungkinan plan.

---

### `path_nodes`

```yaml
path_nodes:
- 1
- 4
- 7
- 8
- 11
```

`path_nodes` adalah jalur node utama yang dilewati robot di dalam forest.

Pada contoh:

```text
1 -> 4 -> 7 -> 8 -> 11
```

Catatan:

- Node `0` adalah posisi luar sebelum masuk forest.
- Node `13` adalah posisi luar setelah keluar forest.
- Node `0` dan `13` biasanya tidak dimasukkan ke `path_nodes`.
- `path_nodes` menunjukkan posisi yang dilewati robot, bukan daftar KFS yang diambil.

---

### `picked_kfs`

```yaml
picked_kfs:
- 4
- 7
- 9
- 11
```

Daftar node KFS yang akan diambil oleh robot.

Pada contoh, robot mengambil KFS di node:

```text
4, 7, 9, 11
```

`picked_kfs` tidak harus sama dengan `path_nodes`.

Contoh:

```yaml
path_nodes:
- 1
- 4
- 7
- 8
- 11

picked_kfs:
- 4
- 7
- 9
- 11
```

Robot bisa berdiri di node `8`, tetapi mengambil KFS di node `9`.

---

## 3. Struktur `ForestPathPoint`

```msg
int32 ZONE_OUTSIDE_ENTRY=0
int32 ZONE_FOREST=1
int32 ZONE_OUTSIDE_EXIT=2

int32 zone
int32 node
int32 anchor_node
```

`ForestPathPoint` digunakan untuk visualisasi path pada GUI atau visualizer.

Contoh:

```yaml
visual_path:
- zone: 0
  node: 0
  anchor_node: 1
- zone: 1
  node: 1
  anchor_node: 1
- zone: 1
  node: 4
  anchor_node: 4
- zone: 2
  node: 13
  anchor_node: 11
```

---

### `zone`

| Nilai | Nama | Arti |
|---:|---|---|
| `0` | `ZONE_OUTSIDE_ENTRY` | Robot masih di luar forest sebelum masuk |
| `1` | `ZONE_FOREST` | Robot berada di dalam forest |
| `2` | `ZONE_OUTSIDE_EXIT` | Robot sudah keluar dari forest |

---

### `node`

Node posisi robot pada visual path.

| Node | Arti |
|---:|---|
| `0` | Posisi luar sebelum masuk forest |
| `1` - `12` | Node di dalam forest |
| `13` | Posisi luar setelah keluar forest |

---

### `anchor_node`

`anchor_node` adalah node forest yang dipakai sebagai referensi visual.

Contoh:

```yaml
- zone: 0
  node: 0
  anchor_node: 1
```

Artinya robot masih di luar forest, tetapi secara visual berada di depan entrance node `1`.

Contoh lain:

```yaml
- zone: 2
  node: 13
  anchor_node: 11
```

Artinya robot sudah keluar forest melalui area exit yang berhubungan dengan node `11`.

---

## 4. Struktur `ForestPlanStep`

```msg
int32 NO_NODE=-1

int32 ACTION_UNKNOWN=0
int32 ACTION_APPROACH=1
int32 ACTION_ENTER=2
int32 ACTION_MOVE=3
int32 ACTION_ROTATE=4
int32 ACTION_PICK_UP=5
int32 ACTION_VALIDATE=6
int32 ACTION_EXIT=7

int32 step

int32 node
int32 target

int32 action
string action_name

string elev
int32 delta

string direction
int32 heading

float64 cost
```

Setiap item pada `steps` adalah satu action yang akan dieksekusi robot.

Catatan penting untuk executor:

- `node` dan `target` tetap dikirim sebagai metadata planner, debugging, visualisasi, dan validasi.
- Untuk action `ENTER`, `PICK_UP`, `MOVE`, `ROTATE`, dan `EXIT`, executor tidak wajib memakai `node` dan `target` sebagai parameter utama gerak.
- Action yang paling membutuhkan `target` secara langsung adalah `APPROACH`, karena `target` menentukan posisi entrance / posisi depan forest yang harus dituju.
- Untuk `APPROACH SHIFT_LEFT` dan `APPROACH SHIFT_RIGHT`, executor cukup memakai `direction` dan `delta` untuk melakukan pergeseran.

---

## 5. Field pada `ForestPlanStep`

### `step`

Nomor urutan action.

Contoh:

```yaml
step: 7
```

Artinya action tersebut adalah step ke-7.

---

### `node`

Node posisi robot saat action dimulai.

Untuk executor, `node` umumnya dipakai sebagai metadata, debugging, dan visualisasi. Executor tidak perlu bergantung pada `node` untuk menjalankan `ENTER`, `PICK_UP`, `MOVE`, `ROTATE`, atau `EXIT`.

Contoh:

```yaml
node: 8
```

Artinya menurut planner, robot sedang berada di node `8` saat step tersebut dibuat.

---

### `target`

Target dari action.

Untuk executor, `target` paling penting pada `APPROACH`.

Makna umum:

- Pada `APPROACH Forward`, `target` adalah entrance forest yang dituju dari martial club.
- Pada `APPROACH SHIFT_LEFT` atau `APPROACH SHIFT_RIGHT`, `target` dapat dipakai sebagai metadata tujuan depan forest, tetapi eksekusi shift cukup memakai `direction` dan `delta`.
- Pada `ENTER`, `PICK_UP`, `MOVE`, `ROTATE`, dan `EXIT`, `target` tetap berguna untuk debugging / visualisasi, tetapi bukan parameter utama executor.

Contoh:

```yaml
target: 1
```

Pada `APPROACH Forward`, ini berarti robot menuju area depan forest entrance node `1`.

---

### `action` dan `action_name`

`action` adalah ID action dalam bentuk angka.

`action_name` adalah nama action dalam bentuk string.

| action | action_name | Fungsi |
|---:|---|---|
| `0` | `UNKNOWN` | Action tidak diketahui |
| `1` | `APPROACH` | Robot menuju atau bergeser di depan forest |
| `2` | `ENTER` | Robot masuk / naik ke forest |
| `3` | `MOVE` | Robot bergerak antar node di dalam forest |
| `4` | `ROTATE` | Robot rotate di tempat |
| `5` | `PICK_UP` | Robot mengambil KFS |
| `6` | `VALIDATE` | Legacy / tidak direkomendasikan sebagai step terpisah |
| `7` | `EXIT` | Robot keluar dari forest |

Catatan:

`ACTION_VALIDATE` masih ada di message untuk compatibility, tetapi pada planner terbaru validate tidak perlu menjadi action step sendiri. Jika ada validate cost, cost tersebut sebaiknya digabung ke `MOVE` atau `ENTER`.

---

### `elev`

`elev` menjelaskan arah perubahan elevasi.

Nilai yang umum:

| elev | Arti |
|---|---|
| `UP` | Robot / mekanisme bergerak naik |
| `DOWN` | Robot / mekanisme bergerak turun |
| `FLAT` | Tidak ada perubahan elevasi |
| `OUTSIDE` | Kondisi khusus saat robot masih di luar forest |
| `-` | Tidak relevan, biasanya pada `ROTATE` |

---

### `delta`

`delta` memiliki arti berbeda tergantung action.

| Action | Arti `delta` |
|---|---|
| `APPROACH` dengan `SHIFT_LEFT` / `SHIFT_RIGHT` | Jumlah blok perpindahan |
| `ENTER` | Jarak naik ke forest dalam mm |
| `PICK_UP` | Jarak naik / turun mekanisme pickup dalam mm |
| `MOVE` | Selisih ketinggian antar node dalam mm |
| `ROTATE` | Biasanya `0` |
| `EXIT` | Jarak turun keluar forest dalam mm |

Untuk `APPROACH SHIFT_LEFT` atau `APPROACH SHIFT_RIGHT`:

```text
delta = 1 -> geser sejauh 1 blok = 120 cm
delta = 2 -> geser sejauh 2 blok = 240 cm
```

Contoh:

```yaml
direction: SHIFT_RIGHT
delta: 2
```

Artinya robot bergeser ke kanan sejauh 2 blok.

```text
2 blok = 2 x 120 cm = 240 cm
```

Contoh lain:

```yaml
direction: SHIFT_LEFT
delta: 1
```

Artinya robot bergeser ke kiri sejauh 1 blok.

```text
1 blok = 120 cm
```

---

### `direction`

`direction` terutama dipakai untuk `APPROACH`.

Untuk `APPROACH`, nilai yang umum:

| direction | Arti |
|---|---|
| `Forward` | Robot dari martial club menuju forest sesuai target entrance |
| `SHIFT_LEFT` | Robot bergeser ke kiri di depan forest |
| `SHIFT_RIGHT` | Robot bergeser ke kanan di depan forest |

Untuk `APPROACH SHIFT_LEFT` dan `APPROACH SHIFT_RIGHT`, arah kiri dan kanan sudah dianggap final dari planner. Executor tidak perlu melakukan mirror lagi antara team blue dan team red.

Untuk `ROTATE`, arah tidak perlu dimirror juga. Executor cukup menjalankan `heading` yang dikirim oleh planner.

Untuk `PICK_UP` dan `MOVE`, executor dapat fokus memakai `elev`, `delta`, dan `heading`.

---

### `heading`

`heading` adalah heading global robot.

| heading | Arti |
|---:|---|
| `0` | Menghadap depan forest |
| `90` | Menghadap kanan |
| `-90` | Menghadap kiri |
| `180` | Menghadap belakang |

`heading` dapat digunakan sebagai koreksi arah sebelum action dijalankan.

Catatan mirror:

- Heading sudah dikirim sebagai heading final dari planner.
- Executor tidak perlu melakukan mirror lagi untuk team blue atau team red.
- `ROTATE` cukup menjalankan heading sesuai nilai parameter.
- Jika `heading: 90`, robot rotate / koreksi ke kanan.
- Jika `heading: -90`, robot rotate / koreksi ke kiri.

---

### `cost`

Cost estimasi untuk step tersebut.

Contoh:

```yaml
cost: 1.5
```

Cost dipakai planner untuk membandingkan efisiensi plan.

---

# 6. Detail Action

## 6.1 `APPROACH`

`APPROACH` digunakan sebelum robot masuk ke forest.

Ada dua jenis approach:

1. `APPROACH Forward`
2. `APPROACH SHIFT_LEFT` / `APPROACH SHIFT_RIGHT`

---

### `APPROACH Forward`

Contoh:

```yaml
- step: 1
  node: 0
  target: 1
  action: 1
  action_name: APPROACH
  elev: OUTSIDE
  delta: 0
  direction: Forward
  heading: 0
  cost: 2.0
```

Artinya robot bergerak dari martial club menuju forest sesuai entrance target.

Pada contoh:

```text
robot dari martial club menuju posisi depan entrance node 1
```

Untuk `APPROACH Forward`, executor memakai `target` untuk menentukan posisi tujuan di depan forest.

Catatan team:

- `APPROACH Forward` dapat memiliki posisi fisik berbeda antara team blue dan team red.
- Misalnya `target: 1` pada team blue dan `target: 1` pada team red bisa memiliki koordinat `x, y` yang berbeda.
- Perbedaan koordinat fisik ini sebaiknya ditangani pada mapping posisi approach di executor / navigation layer.
- Untuk action lain seperti `ROTATE`, `MOVE`, `PICK_UP`, `ENTER`, dan `EXIT`, executor tidak perlu melakukan mirror berdasarkan team jika planner sudah mengirim parameter final.

---

### `APPROACH SHIFT_LEFT` / `APPROACH SHIFT_RIGHT`

Contoh:

```yaml
- step: 1
  node: 0
  target: 3
  action: 1
  action_name: APPROACH
  elev: OUTSIDE
  delta: 2
  direction: SHIFT_RIGHT
  heading: 0
  cost: 1.0
```

Artinya robot bergeser ke kanan di depan forest sejauh `2` blok.

```text
1 blok = 120 cm
2 blok = 240 cm
```

Jadi contoh di atas berarti:

```text
robot melakukan SHIFT_RIGHT sejauh 240 cm
```

Contoh lain:

```yaml
- step: 1
  node: 0
  target: 2
  action: 1
  action_name: APPROACH
  elev: OUTSIDE
  delta: 1
  direction: SHIFT_LEFT
  heading: 0
  cost: 0.5
```

Artinya robot bergeser ke kiri di depan forest sejauh `1` blok.

```text
robot melakukan SHIFT_LEFT sejauh 120 cm
```

Catatan:

- `delta` pada shift selalu berupa jumlah blok.
- Nilai `delta` pada shift umumnya `1` atau `2`.
- `1` blok = `120 cm`.
- Arah `SHIFT_LEFT` dan `SHIFT_RIGHT` sudah final dari planner.
- Executor tidak perlu mirror arah shift untuk team blue atau team red.

---

## 6.2 `ENTER`

Contoh:

```yaml
- step: 2
  node: 0
  target: 1
  action: 2
  action_name: ENTER
  elev: UP
  delta: 400
  direction: Forward
  heading: 0
  cost: 1.8
```

`ENTER` digunakan untuk masuk ke forest.

Secara mekanik, `ENTER` mirip dengan `MOVE`, tetapi khusus untuk naik masuk ke forest.

Parameter utama untuk executor:

| Parameter | Fungsi |
|---|---|
| `elev` | Seharusnya `UP` |
| `delta` | Jarak naik ke forest dalam mm |
| `heading` | Koreksi heading sebelum masuk |

Executor tidak wajib memakai `node` dan `target` untuk menjalankan `ENTER`.

---

## 6.3 `PICK_UP`

Contoh:

```yaml
- step: 9
  node: 8
  target: 9
  action: 5
  action_name: PICK_UP
  elev: DOWN
  delta: 200
  direction: Forward
  heading: 90
  cost: 1.5
```

`PICK_UP` digunakan untuk mengambil KFS.

Parameter utama untuk executor:

| Field | Arti |
|---|---|
| `elev` | Arah mekanisme pickup, bisa `UP` atau `DOWN` |
| `delta` | Jarak naik / turun mekanisme pickup dalam mm |
| `heading` | Koreksi heading sebelum mengambil KFS |

Executor tidak wajib memakai `node` dan `target` untuk menjalankan `PICK_UP`.

Contoh:

```text
mekanisme pickup turun 200 mm
heading robot 90
```

---

## 6.4 `MOVE`

Contoh:

```yaml
- step: 8
  node: 7
  target: 8
  action: 3
  action_name: MOVE
  elev: UP
  delta: 200
  direction: Forward
  heading: 90
  cost: 1.5
```

`MOVE` digunakan untuk berpindah antar node di dalam forest.

Parameter utama untuk executor:

| Field | Arti |
|---|---|
| `elev` | Arah perubahan elevasi, bisa `UP`, `DOWN`, atau `FLAT` |
| `delta` | Selisih ketinggian dalam mm |
| `heading` | Koreksi heading sebelum move |

Executor tidak wajib memakai `node` dan `target` untuk menjalankan `MOVE`.

Contoh:

```text
robot melakukan move dengan elev UP
delta 200 mm
heading 90
```

---

## 6.5 `ROTATE`

Contoh:

```yaml
- step: 7
  node: 7
  target: -1
  action: 4
  action_name: ROTATE
  elev: '-'
  delta: 0
  direction: TURN
  heading: 90
  cost: 0.25
```

`ROTATE` digunakan untuk mengubah heading robot.

Parameter utama untuk executor:

| Field | Arti |
|---|---|
| `heading` | Heading global tujuan rotate |

Executor tidak wajib memakai `node` dan `target` untuk menjalankan `ROTATE`.

Heading:

| heading | Arti |
|---:|---|
| `0` | Depan forest |
| `90` | Kanan |
| `-90` | Kiri |
| `180` | Belakang |

Executor cukup menjalankan heading sesuai parameter. Tidak perlu mirror lagi untuk team blue atau team red.

---

## 6.6 `EXIT`

Contoh:

```yaml
- step: 13
  node: 11
  target: 13
  action: 7
  action_name: EXIT
  elev: DOWN
  delta: 400
  direction: Forward
  heading: 0
  cost: 2.8
```

`EXIT` digunakan untuk keluar dari forest.

Secara mekanik, `EXIT` mirip dengan `MOVE DOWN`.

Parameter utama untuk executor:

| Parameter | Fungsi |
|---|---|
| `elev` | Seharusnya `DOWN` |
| `delta` | Jarak turun keluar forest dalam mm |
| `heading` | Koreksi heading sebelum keluar |

Executor tidak wajib memakai `node` dan `target` untuk menjalankan `EXIT`.

---

# 7. Walkthrough Contoh Plan

Berikut contoh urutan plan:

```text
APPROACH 0 -> 1
ENTER
PICK_UP
MOVE
PICK_UP
MOVE
ROTATE heading 90
MOVE
PICK_UP
ROTATE heading 0
PICK_UP
MOVE
EXIT
```

Penjelasan ringkas:

1. Robot dari martial club menuju posisi depan entrance node `1`.
2. Robot naik masuk ke forest.
3. Robot menjalankan pickup pertama sesuai `elev`, `delta`, dan `heading`.
4. Robot menjalankan move sesuai `elev`, `delta`, dan `heading`.
5. Robot menjalankan pickup kedua sesuai `elev`, `delta`, dan `heading`.
6. Robot menjalankan move berikutnya.
7. Robot rotate ke heading `90`.
8. Robot menjalankan move dengan heading `90`.
9. Robot menjalankan pickup berikutnya.
10. Robot rotate ke heading `0`.
11. Robot menjalankan pickup berikutnya.
12. Robot menjalankan move berikutnya.
13. Robot keluar forest.

---

# 8. Catatan Implementasi Executor

Executor bisa menggunakan field berikut sebagai parameter utama:

| Action | Parameter utama |
|---|---|
| `APPROACH Forward` | `target`, `direction` |
| `APPROACH SHIFT_LEFT/SHIFT_RIGHT` | `direction`, `delta` |
| `ENTER` | `elev`, `delta`, `heading` |
| `PICK_UP` | `elev`, `delta`, `heading` |
| `MOVE` | `elev`, `delta`, `heading` |
| `ROTATE` | `heading` |
| `EXIT` | `elev`, `delta`, `heading` |

Catatan:

- Untuk `APPROACH Forward`, `target` dipakai untuk memilih posisi depan forest yang akan dituju.
- Posisi fisik `APPROACH Forward` dapat berbeda antara team blue dan team red, sehingga mapping koordinat `x, y` approach dapat mempertimbangkan team.
- Untuk `APPROACH SHIFT_LEFT/SHIFT_RIGHT`, `delta` adalah jumlah blok.
- Nilai `delta` untuk shift umumnya `1` atau `2`.
- Satu blok approach adalah `120 cm`.
- Arah `SHIFT_LEFT` dan `SHIFT_RIGHT` sudah final dari planner, jadi executor tidak perlu mirror lagi.
- Untuk `ROTATE`, executor cukup menjalankan `heading` global yang dikirim planner.
- Untuk `ENTER`, `elev` seharusnya `UP`.
- Untuk `EXIT`, `elev` seharusnya `DOWN`.
- Untuk `PICK_UP`, `elev` dapat berupa `UP` atau `DOWN`.
- Untuk `MOVE`, `elev` dapat berupa `UP`, `DOWN`, atau `FLAT`.
- `node` dan `target` tetap berguna untuk debugging, visualisasi, logging, dan validasi planner.