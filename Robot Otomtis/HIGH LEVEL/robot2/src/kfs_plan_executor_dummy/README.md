# kfs_plan_executor_dummy

Package dummy executor baru untuk membaca `/intelligent/kfs_retrieval_plan` dan mengeksekusi log saat `/gui_data.start == true`.

Nama package ini sengaja berbeda dari package dummy sebelumnya:

```text
kfs_plan_executor_dummy
```

## Kenapa dibuat aman

- Single C++ file, tidak ada static library terpisah, jadi menghindari `undefined reference`.
- Tidak memakai `shared_from_this()` di constructor, jadi menghindari `std::bad_weak_ptr`.
- `start=true` hanya execute sekali sampai start kembali false.
- `VALIDATE` tidak crash, hanya warning dan skip.
- Subscribe ke topic yang sama:
  - `/intelligent/kfs_retrieval_plan`
  - `/gui_data`

## Install

Extract folder ini ke workspace `src`:

```bash
cd /home/eilero/robot2/src
unzip kfs_plan_executor_dummy.zip
```

Build:

```bash
cd /home/eilero/robot2
colcon build --packages-select kfs_plan_executor_dummy --symlink-install
source install/setup.bash
```

Cek:

```bash
ros2 pkg list | grep kfs_plan_executor_dummy
```

Launch:

```bash
ros2 launch kfs_plan_executor_dummy kfs_plan_executor_dummy.launch.py
```
