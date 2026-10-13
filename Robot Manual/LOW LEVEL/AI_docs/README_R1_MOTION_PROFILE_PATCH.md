# R1 Motion Profile + Arena Mirror Patch

Patch minimal ini menambahkan layer gerak R1 di `main.c` tanpa membuat `Sequence.*`.

## File yang berubah

- `Core/Src/main.c`
- `Core/Inc/R1PathPlanner.h` hanya dirapikan kalau ada typo duplicate `R1Side_t`.

## Fitur baru

### 1. Set arena mode

```c
R1_SetArenaMode(R1_ARENA_BLUE);
R1_SetArenaMode(R1_ARENA_RED);
```

### 2. Wrapper koordinat BASE

Tulis sequence dengan koordinat BASE BLUE. Saat `R1_ARENA_RED`, X dan heading otomatis di-mirror lewat `R1Path_ToArenaX()` dan `R1Path_ToArenaH()`.

```c
R1_GerakPutarHeadingBase(90.0f);
R1_GerakJalanKeTitikBase(800.0f, 1200.0f, 90.0f);
R1_GerakJalanKeTitikFinalHeadingBase(800.0f, 1200.0f, 90.0f);
```

### 3. Motion profile

Preset yang ditambah:

```c
R1_MOVE_NORMAL
R1_MOVE_PRECISE
R1_MOVE_FAST
```

Parameter baru lebih mudah dibaca:

```c
max_output_speed
start_output_speed
arrive_distance_mm
arrive_heading_deg
pid_distance_limit_mm
pid_heading_limit_deg
kp_distance
kd_distance
kp_heading
kd_heading
```

Mapper ke variable lama:

```c
R1_ApplyMotionProfile(&R1_MOVE_NORMAL);
```

### 4. Wrapper final heading + profile

```c
R1_GerakJalanKeTitikFinalHeadingProfile(x, y, h, &R1_MOVE_PRECISE);

R1_GerakJalanKeTitikFinalHeadingBaseProfile(x_base,
                                            y_mm,
                                            h_base,
                                            &R1_MOVE_PRECISE);
```

## Contoh test di `MISC_SHARE`

Patch ini juga mengganti test `MISC_SHARE` agar memakai wrapper baru:

```c
SetWorldPosition_RIZAL(0.0f, 0.0f, 0.0f);

R1_SetArenaMode(R1_ARENA_BLUE);
/* R1_SetArenaMode(R1_ARENA_RED); */

R1_GerakJalanKeTitikFinalHeadingBaseProfile(0.0f,
                                            1000.0f,
                                            90.0f,
                                            &R1_MOVE_NORMAL);
```

Untuk test arena merah, aktifkan:

```c
R1_SetArenaMode(R1_ARENA_RED);
```

lalu target BASE yang sama akan otomatis menjadi heading mirror.
