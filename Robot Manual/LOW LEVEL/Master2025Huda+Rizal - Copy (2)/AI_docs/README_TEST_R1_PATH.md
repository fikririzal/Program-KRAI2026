# R1PathPlanner STM32 v4 - Approach Mode

Library ini hanya **generate waypoint path** untuk R1 menuju depan Meihua Forest.
Library tidak menjalankan motor dan tidak memanggil `SetWorldPosition()`.

## File

```text
Core/Inc/R1PathPlanner.h
Core/Src/R1PathPlanner.c
main_task_snippet.txt
```

## Mode baru v4

v4 menambahkan `R1ApproachMode_t`:

```c
R1_APPROACH_NORMAL
R1_APPROACH_AVOID_R2_ZONE
```

### R1_APPROACH_NORMAL

Mapping target:

```text
MF 1,2,3      -> R1_SIDE_TOP
MF 4,7        -> R1_SIDE_LEFT
MF 6,9        -> R1_SIDE_RIGHT
MF 10,11,12   -> R1_SIDE_BOTTOM
```

### R1_APPROACH_AVOID_R2_ZONE

Mode ini mengurangi R1 berada di sisi TOP/BOTTOM yang dekat area R2 Entrance / R2 Pathway.

Mapping target:

```text
MF 1,4,7,10   -> R1_SIDE_LEFT
MF 2          -> R1_SIDE_TOP
MF 3,6,9,12   -> R1_SIDE_RIGHT
MF 11         -> R1_SIDE_BOTTOM
```

Dengan mode ini:

```text
R1_SIDE_TOP hanya untuk MF 2
R1_SIDE_BOTTOM hanya untuk MF 11
```

## Cara test Live Expressions

Panggil berkala di task:

```c
R1Path_DebugGenerateTask();
```

Set di Live Expressions:

```c
R1_CFG_ARENA_MODE = R1_ARENA_RED;          // atau R1_ARENA_BLUE
R1_DBG_ApproachMode = R1_APPROACH_AVOID_R2_ZONE;
R1_DBG_TargetBox = 1;
R1_DBG_HeadingMode = R1_HEADING_HADAP_FOREST_SEBELUM_JALAN;
R1_DBG_RequestGenerate = 1;
```

Cek:

```c
R1_DBG_PathCount
R1_DBG_CurrentSide
R1_DBG_TargetSide
R1_DBG_PathX[0]
R1_DBG_PathY[0]
R1_DBG_PathH[0]
R1_DBG_PathSide[0]
```

## Expected contoh

Untuk RED + `R1_APPROACH_AVOID_R2_ZONE` + target MF 1 dari OUTSIDE:

```text
Path target akhir:
x    = -200
y    = 2300
h    = -90
side = R1_SIDE_LEFT
```

Untuk BLUE + `R1_APPROACH_AVOID_R2_ZONE` + target MF 1 dari OUTSIDE:

```text
Path target akhir:
x    = 200
y    = 2300
h    = 90
side = R1_SIDE_LEFT
```

Untuk RED + `R1_APPROACH_AVOID_R2_ZONE` + target MF 2 dari OUTSIDE:

```text
PathCount = 1
x    = -2500
y    = 1200
h    = 0
side = R1_SIDE_TOP
```

## Catatan penting

- `R1_DBG_ApproachMode` otomatis disalin ke `R1_CFG_APPROACH_MODE` saat `R1Path_DebugGenerateTask()` melakukan generate.
- Kalau generate manual tanpa debug task, set langsung `R1_CFG_APPROACH_MODE` sebelum panggil `R1Path_GenerateToMeihua()`.
- Path index setelah `R1_DBG_PathCount - 1` harus diabaikan.
