# KFS Retrieval Planner Cost Configuration

README ini menjelaskan konfigurasi cost yang digunakan planner untuk menghitung estimasi biaya setiap action pada KFS Retrieval.

File cost ini berfungsi sebagai **physical cost matrix**, yaitu kumpulan nilai estimasi untuk:

- approach menuju forest
- pergerakan naik / turun
- rotasi robot
- manipulasi KFS
- validasi KFS R1
- bias exit
- penalti kecil untuk mengurangi zig-zag

Cost ini tidak harus sama persis dengan waktu real robot, tetapi digunakan planner untuk memilih rute yang lebih efisien.

---

## 1. Konsep Umum Cost

Planner membandingkan beberapa kemungkinan plan berdasarkan `total_cost`.

Secara sederhana:

```text
total_cost = jumlah seluruh cost dari setiap step
```

Contoh:

```text
APPROACH cost + ENTER cost + PICK_UP cost + MOVE cost + EXIT cost
```

Semakin kecil `total_cost`, plan tersebut dianggap lebih efisien oleh planner.

---

## 2. Approach Costs

```python
C_APPROACH_INITIAL = {
    1: 2.00,
    2: 1.75,
    3: 1.50,
}

C_APPROACH_SHIFT_PER_NODE = 0.5
```

Cost approach digunakan saat robot masih berada di luar forest.

Ada dua jenis approach:

1. `APPROACH Forward`
2. `APPROACH SHIFT_LEFT` / `APPROACH SHIFT_RIGHT`

---

### 2.1 `C_APPROACH_INITIAL`

```python
C_APPROACH_INITIAL = {
    1: 2.00,
    2: 1.75,
    3: 1.50,
}
```

Cost ini digunakan ketika robot dari martial club langsung menuju posisi depan forest sesuai entrance target.

| Target entrance | Cost |
|---:|---:|
| `1` | `2.00` |
| `2` | `1.75` |
| `3` | `1.50` |

Contoh:

```yaml
action_name: APPROACH
direction: Forward
target: 1
cost: 2.00
```

Artinya robot bergerak dari martial club menuju posisi depan entrance node `1`.

---

### 2.2 `C_APPROACH_SHIFT_PER_NODE`

```python
C_APPROACH_SHIFT_PER_NODE = 0.5
```

Cost ini digunakan saat robot sudah berada di depan forest lalu bergeser ke kiri atau kanan.

```text
approach_shift_cost = delta * C_APPROACH_SHIFT_PER_NODE
```

`delta` adalah jumlah blok pergeseran.

```text
delta = 1 -> geser 1 blok = 120 cm
delta = 2 -> geser 2 blok = 240 cm
```

Contoh:

```text
delta = 2
C_APPROACH_SHIFT_PER_NODE = 0.5

cost = 2 * 0.5 = 1.0
```

---

## 3. Elevation Costs

```python
C_ELEV_UP_200 = 1.5
C_ELEV_UP_400 = 1.8
C_ELEV_DOWN_200 = 1.7
C_ELEV_DOWN_400 = 2.0
```

Cost elevasi digunakan untuk action yang melibatkan perubahan tinggi robot, terutama:

- `ENTER`
- `MOVE`
- `EXIT`

| Kondisi | Cost |
|---|---:|
| Naik 200 mm | `1.5` |
| Naik 400 mm | `1.8` |
| Turun 200 mm | `1.7` |
| Turun 400 mm | `2.0` |

---

### 3.1 Move Up

Jika robot bergerak naik 200 mm:

```text
elev = UP
delta = 200
cost = C_ELEV_UP_200 = 1.5
```

Jika robot bergerak naik 400 mm:

```text
elev = UP
delta = 400
cost = C_ELEV_UP_400 = 1.8
```

---

### 3.2 Move Down

Jika robot bergerak turun 200 mm:

```text
elev = DOWN
delta = 200
cost = C_ELEV_DOWN_200 = 1.7
```

Jika robot bergerak turun 400 mm:

```text
elev = DOWN
delta = 400
cost = C_ELEV_DOWN_400 = 2.0
```

---

## 4. Rotation Costs

```python
C_ROT_90 = 0.25
C_ROT_180 = 0.50
```

Cost rotasi digunakan saat robot perlu mengubah heading.

| Rotasi | Cost |
|---|---:|
| 90 derajat | `0.25` |
| 180 derajat | `0.50` |

Heading global robot:

| Heading | Arti |
|---:|---|
| `0` | Depan forest |
| `90` | Kanan |
| `-90` | Kiri |
| `180` | Belakang |

Contoh:

```text
robot heading 0
target heading 90

angle_diff = 90
rotate cost = C_ROT_90 = 0.25
```

Contoh lain:

```text
robot heading 0
target heading 180

angle_diff = 180
rotate cost = C_ROT_180 = 0.50
```

---

## 5. KFS Manipulation Costs

```python
C_PICK_ELEV_UP = 0.10
C_PICK_ELEV_DOWN = 0.30
C_PICK_DIR_FORWARD = 1.20
```

Cost ini digunakan untuk action `PICK_UP`.

Dengan aturan terbaru, pickup difokuskan sebagai pickup forward. Jika robot perlu mengambil KFS yang tidak berada di arah depan, planner sebaiknya membuat action `ROTATE` terlebih dahulu, lalu `PICK_UP`.

---

### 5.1 `C_PICK_ELEV_UP`

```python
C_PICK_ELEV_UP = 0.10
```

Tambahan cost ketika mekanisme pickup bergerak naik.

Contoh:

```text
elev = UP
delta = 200
```

Pickup elevation cost:

```text
0.10
```

---

### 5.2 `C_PICK_ELEV_DOWN`

```python
C_PICK_ELEV_DOWN = 0.30
```

Tambahan cost ketika mekanisme pickup bergerak turun.

Contoh:

```text
elev = DOWN
delta = 200
```

Pickup elevation cost:

```text
0.30
```

---

### 5.3 `C_PICK_DIR_FORWARD`

```python
C_PICK_DIR_FORWARD = 1.20
```

Base cost untuk melakukan pickup.

Rumus pickup cost:

```text
pickup_cost = C_PICK_DIR_FORWARD + C_PICK_ELEV_UP/DOWN
```

Contoh pickup up:

```text
C_PICK_DIR_FORWARD = 1.20
C_PICK_ELEV_UP = 0.10

pickup_cost = 1.20 + 0.10 = 1.30
```

Contoh pickup down:

```text
C_PICK_DIR_FORWARD = 1.20
C_PICK_ELEV_DOWN = 0.30

pickup_cost = 1.20 + 0.30 = 1.50
```

---

## 6. Validation Costs

```python
C_VALIDATE_UP = 0.01
C_VALIDATE_DOWN = 0.01

C_VALIDATE_ROW_PENALTY = {
    0: 0.50,
    1: 0.30,
    2: 0.10,
    3: 0.05,
}
```

Validation cost digunakan ketika robot melewati atau masuk ke node yang berisi KFS R1.

KFS R1 tidak dianggap sebagai obstacle keras, tetapi melewati node tersebut diberi cost tambahan karena perlu validasi.

Validate tidak perlu menjadi action step sendiri. Cost validate sebaiknya ditempel ke action `MOVE` atau `ENTER`.

---

### 6.1 Base Validate Cost

```python
C_VALIDATE_UP = 0.01
C_VALIDATE_DOWN = 0.01
```

Base cost validasi berdasarkan arah elevasi.

| Elevasi | Base validate cost |
|---|---:|
| `UP` | `0.01` |
| `DOWN` | `0.01` |

Nilai ini kecil karena cost utama validasi dibedakan berdasarkan baris / row.

---

### 6.2 Row Validate Penalty

```python
C_VALIDATE_ROW_PENALTY = {
    0: 0.50,  # node 1, 2, 3
    1: 0.30,  # node 4, 5, 6
    2: 0.10,  # node 7, 8, 9
    3: 0.05,  # node 10, 11, 12
}
```

Penalty ini membuat validasi di baris depan lebih mahal daripada baris belakang.

Pembagian row:

| Row | Node | Penalty |
|---:|---|---:|
| `0` | `1, 2, 3` | `0.50` |
| `1` | `4, 5, 6` | `0.30` |
| `2` | `7, 8, 9` | `0.10` |
| `3` | `10, 11, 12` | `0.05` |

Semakin depan posisi KFS R1, semakin besar penalty validate.

---

### 6.3 Rumus Validate Cost

```text
validate_cost = C_VALIDATE_UP/DOWN + C_VALIDATE_ROW_PENALTY[row]
```

Contoh validasi node `2` dengan elevasi `UP`:

```text
node 2 berada di row 0

base validate up = 0.01
row penalty = 0.50

validate_cost = 0.01 + 0.50 = 0.51
```

Contoh validasi node `6` dengan elevasi `DOWN`:

```text
node 6 berada di row 1

base validate down = 0.01
row penalty = 0.30

validate_cost = 0.01 + 0.30 = 0.31
```

Contoh validasi node `11` dengan elevasi `DOWN`:

```text
node 11 berada di row 3

base validate down = 0.01
row penalty = 0.05

validate_cost = 0.01 + 0.05 = 0.06
```

---

## 7. Exit Bias Costs

```python
C_EXIT_NODE_10 = 0.30
C_EXIT_NODE_11 = 0.80
C_EXIT_NODE_12 = 0.80
```

Exit bias cost adalah tambahan cost ketika robot keluar forest dari node tertentu.

| Exit dari node | Tambahan cost |
|---:|---:|
| `10` | `0.30` |
| `11` | `0.80` |
| `12` | `0.80` |

Cost ini digunakan untuk memberi preferensi terhadap jalur keluar tertentu.

Contoh exit dari node `10`:

```text
base exit down cost = C_ELEV_DOWN_400 = 2.0
exit bias node 10 = 0.30

exit cost = 2.0 + 0.30 = 2.30
```

Contoh exit dari node `11`:

```text
base exit down cost = C_ELEV_DOWN_400 = 2.0
exit bias node 11 = 0.80

exit cost = 2.0 + 0.80 = 2.80
```

---

## 8. Node Penalty

```python
C_NODE_PENALTY = 0.2
```

Penalty kecil untuk mengurangi zig-zag atau gerakan yang terlalu banyak melewati node.

Nilai ini biasanya digunakan dalam path search, terutama untuk membantu planner memilih path yang lebih sederhana.

Contoh:

```text
MOVE base cost = 1.5
node penalty = 0.2

step search cost = 1.5 + 0.2 = 1.7
```

Catatan:

`C_NODE_PENALTY` bisa digunakan pada search cost untuk membandingkan jalur, tetapi tidak selalu harus muncul sebagai cost action final jika action final sudah memiliki cost fisik tersendiri.

---

## 9. Emergency Extra-Pick Threshold

```python
THRESHOLD_EXTRA_PICKUP_FOR_EXPENSIVE_EXIT = 2.0
```

Threshold ini dipakai untuk menentukan apakah planner layak mengambil KFS tambahan ketika jalur exit dianggap mahal.

Secara konsep:

```text
jika exit langsung terlalu mahal,
planner boleh mempertimbangkan pickup tambahan
selama tambahan cost masih masuk threshold
```

Nilai:

```text
2.0
```

berarti planner masih bisa menerima tambahan pickup jika cost tambahan tersebut tidak terlalu besar dibanding manfaatnya.

---

## 10. Contoh Perhitungan Cost

### Contoh 1 — Pick Up UP

```text
C_PICK_DIR_FORWARD = 1.20
C_PICK_ELEV_UP = 0.10

pickup_cost = 1.20 + 0.10 = 1.30
```

---

### Contoh 2 — Pick Up DOWN

```text
C_PICK_DIR_FORWARD = 1.20
C_PICK_ELEV_DOWN = 0.30

pickup_cost = 1.20 + 0.30 = 1.50
```

---

### Contoh 3 — Move UP 200 mm

```text
C_ELEV_UP_200 = 1.5

move_cost = 1.5
```

Jika dipakai dalam search dan ditambah node penalty:

```text
move_search_cost = 1.5 + C_NODE_PENALTY
move_search_cost = 1.5 + 0.2 = 1.7
```

---

### Contoh 4 — Move melewati R1 di node 6

Misalnya robot move ke node `6`, dan node `6` berisi KFS R1.

```text
move base cost = C_ELEV_DOWN_200 = 1.7
validate base down = C_VALIDATE_DOWN = 0.01
node 6 berada di row 1
row penalty = 0.30

validate_cost = 0.01 + 0.30 = 0.31

final cost = 1.7 + 0.31 = 2.01
```

---

### Contoh 5 — Exit dari Node 11

```text
base exit down cost = C_ELEV_DOWN_400 = 2.0
exit bias node 11 = C_EXIT_NODE_11 = 0.80

exit cost = 2.0 + 0.80 = 2.80
```

---

## 11. Ringkasan Parameter

| Parameter | Nilai | Fungsi |
|---|---:|---|
| `C_APPROACH_INITIAL[1]` | `2.00` | Cost approach awal ke entrance 1 |
| `C_APPROACH_INITIAL[2]` | `1.75` | Cost approach awal ke entrance 2 |
| `C_APPROACH_INITIAL[3]` | `1.50` | Cost approach awal ke entrance 3 |
| `C_APPROACH_SHIFT_PER_NODE` | `0.5` | Cost shift per blok |
| `C_ELEV_UP_200` | `1.5` | Cost naik 200 mm |
| `C_ELEV_UP_400` | `1.8` | Cost naik 400 mm |
| `C_ELEV_DOWN_200` | `1.7` | Cost turun 200 mm |
| `C_ELEV_DOWN_400` | `2.0` | Cost turun 400 mm |
| `C_ROT_90` | `0.25` | Cost rotate 90 derajat |
| `C_ROT_180` | `0.50` | Cost rotate 180 derajat |
| `C_PICK_ELEV_UP` | `0.10` | Tambahan cost pickup naik |
| `C_PICK_ELEV_DOWN` | `0.30` | Tambahan cost pickup turun |
| `C_PICK_DIR_FORWARD` | `1.20` | Base cost pickup |
| `C_VALIDATE_UP` | `0.01` | Base cost validate saat elevasi up |
| `C_VALIDATE_DOWN` | `0.01` | Base cost validate saat elevasi down |
| `C_EXIT_NODE_10` | `0.30` | Bias exit dari node 10 |
| `C_EXIT_NODE_11` | `0.80` | Bias exit dari node 11 |
| `C_EXIT_NODE_12` | `0.80` | Bias exit dari node 12 |
| `C_NODE_PENALTY` | `0.2` | Penalty kecil per node untuk search |
| `THRESHOLD_EXTRA_PICKUP_FOR_EXPENSIVE_EXIT` | `2.0` | Threshold extra pickup saat exit mahal |

---

## 12. Catatan Tuning

Beberapa parameter yang paling sering perlu dituning:

### Approach

Jika robot lebih susah menuju entrance tertentu, naikkan nilai pada:

```python
C_APPROACH_INITIAL
```

Jika shift kiri / kanan terlalu sering dipilih, naikkan:

```python
C_APPROACH_SHIFT_PER_NODE
```

---

### Elevation

Jika naik lebih susah daripada turun, naikkan:

```python
C_ELEV_UP_200
C_ELEV_UP_400
```

Jika turun lebih susah daripada naik, naikkan:

```python
C_ELEV_DOWN_200
C_ELEV_DOWN_400
```

---

### Rotation

Jika planner terlalu sering rotate, naikkan:

```python
C_ROT_90
C_ROT_180
```

Jika planner terlalu menghindari rotate padahal rotate aman, turunkan nilai tersebut.

---

### Pickup

Jika planner terlalu sering mengambil KFS tambahan, naikkan:

```python
C_PICK_DIR_FORWARD
C_PICK_ELEV_UP
C_PICK_ELEV_DOWN
```

Jika pickup terlalu dihindari, turunkan nilai tersebut.

---

### Validate R1

Jika planner terlalu sering lewat KFS R1 depan, naikkan penalty row depan:

```python
C_VALIDATE_ROW_PENALTY[0]
```

Jika planner terlalu menghindari semua R1, turunkan:

```python
C_VALIDATE_UP
C_VALIDATE_DOWN
C_VALIDATE_ROW_PENALTY
```

---

### Exit

Jika planner terlalu sering keluar dari node tertentu, naikkan bias exit node tersebut.

Contoh:

```python
C_EXIT_NODE_11 = 1.0
```

Jika planner terlalu menghindari node exit tertentu, turunkan bias exit node tersebut.

