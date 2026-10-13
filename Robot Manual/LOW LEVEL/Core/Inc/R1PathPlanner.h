#ifndef R1_PATH_PLANNER_H
#define R1_PATH_PLANNER_H

/*
 * ============================================================
 * R1PathPlanner v4 - ABU Robocon 2026 R1 Meihua Pathway Planner
 * ============================================================
 *
 * Fungsi library:
 * - Generate waypoint aman untuk R1 menuju depan box Meihua Forest.
 * - Tidak menjalankan motor.
 * - Tidak memanggil SetWorldPosition().
 * - Membaca posisi robot dari global variable:
 *      PosX_World
 *      PosY_World
 *      RobotYaw
 *
 * Catatan:
 * - R1 hanya bergerak di R1 Pathway.
 * - Forest block hijau/elevated tidak dilewati R1.
 * - R1_SIDE_OUTSIDE digunakan saat robot masih di Start Zone,
 *   Martial Club, Staff Rack, atau area transisi sebelum masuk pathway.
 * - v4 menambahkan R1ApproachMode_t untuk memilih cara ambil box:
 *      NORMAL       : 1/2/3 dari TOP, 10/11/12 dari BOTTOM.
 *      AVOID_R2_ZONE: TOP hanya untuk 2, BOTTOM hanya untuk 11.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================
 * CONFIG CONSTANT
 * ============================================================
 *
 * Base coordinate:
 * - Semua nilai BASE dibuat untuk arena BLUE.
 * - Untuk arena RED, X otomatis dimirror menjadi negatif.
 *
 * Sistem koordinat robot:
 * - X+ = kanan robot saat start.
 * - Y+ = maju dari start ke arah Meihua Forest.
 * - H 0 = menghadap forest dari sisi TOP.
 */

#define R1_MAX_PATH                         8u

#define R1_BLOCK_SIZE_MM                    1200.0f

/*
 * Ukuran total/legal robot.
 * Ini dipakai sebagai envelope umum, bukan selalu sebagai footprint tabrakan.
 */
#define R1_ROBOT_WIDTH_X_MM                   800.0f
#define R1_ROBOT_LENGTH_Y_MM                 1000.0f
#define R1_ROBOT_HALF_WIDTH_X_MM             (R1_ROBOT_WIDTH_X_MM  * 0.5f)
#define R1_ROBOT_HALF_LENGTH_Y_MM            (R1_ROBOT_LENGTH_Y_MM * 0.5f)

/*
 * Footprint base bawah robot.
 * Planner pathway lebih aman dihitung dari base yang bisa mentok tembok/riser,
 * bukan dari mekanisme belakang yang boleh overhang di atas tembok rendah.
 *
 * Nilai aktual terbaru:
 * - R1_CENTER_TO_BASE_FRONT_MM = 350 mm.
 * - R1_CENTER_TO_BASE_REAR_MM  = 290 mm, permukaan rear base yang menyentuh jig.
 * - kiri/kanan base saat start diasumsikan simetris 400 mm.
 *
 * Catatan penting:
 * - Di R1_SIDE_LEFT, robot heading world 90 deg sehingga depan robot menghadap
 *   forest dan belakang robot menghadap tembok kiri.
 * - X safe LEFT sekarang di-center-kan di koridor kiri berdasarkan panjang base
 *   aktual FRONT+REAR, bukan hanya didekatkan ke forest.
 */
#define R1_BASE_WIDTH_X_MM                    800.0f
#define R1_BASE_LENGTH_Y_MM                   820.0f

#define R1_CENTER_TO_BASE_FRONT_MM            350.0f
#define R1_CENTER_TO_BASE_REAR_MM             290.0f
#define R1_CENTER_TO_BASE_LEFT_MM             (R1_BASE_WIDTH_X_MM * 0.5f)
#define R1_CENTER_TO_BASE_RIGHT_MM            (R1_BASE_WIDTH_X_MM * 0.5f)

/* Mekanisme belakang total. Dipakai sebagai referensi konfigurasi, bukan routing. */
#define R1_CENTER_TO_MECH_REAR_MM             R1_ROBOT_HALF_LENGTH_Y_MM

#define R1_PATHWAY_CLEARANCE_MM                50.0f

/*
 * Jig start BLUE:
 * - Robot di-start dengan jig di belakang agar mekanisme tidak keluar lapangan.
 * - Titik zero tetap titik tengah robot.
 * - Center Y dari tembok belakang = jig belakang + jarak center ke belakang base.
 *
 * Jika hasil zero lapangan bergeser, tuning hanya dua nilai ini:
 *   R1_START_JIG_REAR_SPACER_Y_MM
 *   R1_CENTER_TO_BASE_REAR_MM
 *
 * Jangan mengubah fungsi SetWorldPosition_RIZAL(). Cukup panggil:
 *   SetWorldPosition_RIZAL(R1_START_ZERO_X_MM,
 *                          R1_START_ZERO_Y_MM,
 *                          R1_START_ZERO_H_DEG);
 */
#define R1_START_JIG_LENGTH_X_MM              800.0f
#define R1_START_JIG_REAR_SPACER_Y_MM         180.0f

#define R1_START_CENTER_X_FROM_LEFT_WALL_MM   R1_CENTER_TO_BASE_LEFT_MM
#define R1_START_CENTER_Y_FROM_BACK_WALL_MM   (R1_START_JIG_REAR_SPACER_Y_MM + \
                                               R1_CENTER_TO_BASE_REAR_MM)

#define R1_START_ZERO_X_MM                     0.0f
#define R1_START_ZERO_Y_MM                     0.0f
#define R1_START_ZERO_H_DEG                    0.0f

/*
 * Offset center robot dari sisi forest/pathway per sisi.
 * Untuk TOP/LEFT/RIGHT robot menghadap forest, jadi clearance dihitung dari
 * permukaan depan base. BOTTOM dibuat konservatif memakai envelope lama.
 */
#define R1_PICK_OFFSET_TOP_MM                 (R1_CENTER_TO_BASE_FRONT_MM + R1_PATHWAY_CLEARANCE_MM)
#define R1_PICK_OFFSET_LEFT_MM                (R1_CENTER_TO_BASE_FRONT_MM + R1_PATHWAY_CLEARANCE_MM)
#define R1_PICK_OFFSET_RIGHT_MM               (R1_CENTER_TO_BASE_FRONT_MM + R1_PATHWAY_CLEARANCE_MM)
#define R1_PICK_OFFSET_BOTTOM_MM              (R1_ROBOT_HALF_LENGTH_Y_MM + R1_PATHWAY_CLEARANCE_MM)

/*
 * Safe line berdasarkan FINAL side robot.
 *
 * LEFT khusus di-center-kan di koridor kiri:
 * - Saat final side = R1_SIDE_LEFT, heading robot = 90 deg.
 * - Depan base menghadap forest (+X).
 * - Rear base menghadap tembok kiri / outer wall (-X).
 * - Karena itu X center dihitung dari dua batas: tembok kiri dan sisi kiri forest.
 *
 * Rumus LEFT centered:
 *   clear_wall   = (x_center - rear_base) - left_outer_wall
 *   clear_forest = forest_left - (x_center + front_base)
 *   clear_wall == clear_forest
 *
 * Dengan data sekarang:
 *   left_outer_wall = -400
 *   forest_left     = 700
 *   front_base      = 350
 *   rear_base       = 290
 *   x_center_left   = 120
 *
 * RIGHT sementara tetap memakai offset depan base dari forest karena data outer
 * wall kanan belum dimasukkan eksplisit ke planner.
 * TOP memakai center -> FRONT BASE.
 * BOTTOM tetap konservatif memakai envelope bottom.
 */
#define R1_LEFT_OUTER_WALL_X_BASE             (-R1_START_CENTER_X_FROM_LEFT_WALL_MM)
#define R1_BASE_LENGTH_FRONT_REAR_MM          (R1_CENTER_TO_BASE_FRONT_MM + R1_CENTER_TO_BASE_REAR_MM)
#define R1_LEFT_PATHWAY_WIDTH_MM              (R1_FOREST_LEFT_X_BASE - R1_LEFT_OUTER_WALL_X_BASE)
#define R1_LEFT_PATHWAY_CLEARANCE_CENTER_MM   ((R1_LEFT_PATHWAY_WIDTH_MM - R1_BASE_LENGTH_FRONT_REAR_MM) * 0.5f)

#define R1_SAFE_X_LEFT_BASE                   (R1_LEFT_OUTER_WALL_X_BASE + \
                                               R1_CENTER_TO_BASE_REAR_MM + \
                                               R1_LEFT_PATHWAY_CLEARANCE_CENTER_MM)
#define R1_SAFE_X_RIGHT_BASE                  (R1_FOREST_RIGHT_X_BASE + R1_PICK_OFFSET_RIGHT_MM)
#define R1_SAFE_Y_TOP                         (R1_FOREST_TOP_Y        - R1_PICK_OFFSET_TOP_MM)
#define R1_SAFE_Y_BOTTOM                      (R1_FOREST_BOTTOM_Y     + R1_PICK_OFFSET_BOTTOM_MM)

/* Kompatibilitas untuk kode lama yang masih membaca R1_PICK_OFFSET_MM. */
#define R1_PICK_OFFSET_MM                     R1_PICK_OFFSET_TOP_MM

#define R1_START_TO_ZONE2_Y_MM      1000.0f   // 970 + 30
#define R1_ZONE2_TO_FOREST_Y_MM     1200.0f

/*
 * Contoh hasil dengan start zero di tengah robot dan data base terbaru:
 * - R1_FOREST_TOP_Y efektif = 2730 mm.
 * - BLUE box 1 TOP front    = +1300, 2330
 * - BLUE box 1 LEFT center  = +120,  3330
 *
 * Catatan:
 * - TOP front dipakai kalau approach dari sisi TOP.
 * - LEFT center dipakai kalau mode avoid R2 memaksa box 1 diambil dari sisi LEFT.
 */
#define R1_START_ZONE_DEPTH_Y_MM            1000.0f
#define R1_FOREST_LEFT_X_BASE                700.0f
#define R1_CENTER_TO_START_FRONT_Y_MM        (R1_START_ZONE_DEPTH_Y_MM - \
                                              R1_START_CENTER_Y_FROM_BACK_WALL_MM)

#define R1_FOREST_TOP_Y                      (R1_CENTER_TO_START_FRONT_Y_MM + \
                                              R1_START_TO_ZONE2_Y_MM + \
                                              R1_ZONE2_TO_FOREST_Y_MM)

#define R1_FOREST_RIGHT_X_BASE              (R1_FOREST_LEFT_X_BASE + (3.0f * R1_BLOCK_SIZE_MM))
#define R1_FOREST_BOTTOM_Y                  (R1_FOREST_TOP_Y        + (4.0f * R1_BLOCK_SIZE_MM))

/*
 * Optional entry tengah TOP.
 * Ini tidak wajib dipakai oleh generator v3.
 * Disediakan kalau nanti kamu ingin mode conservative-entry.
 */
#define R1_TOP_ENTRY_CENTER_X_BASE          (R1_FOREST_LEFT_X_BASE + (1.5f * R1_BLOCK_SIZE_MM))
#define R1_TOP_ENTRY_CENTER_Y               R1_SAFE_Y_TOP

/*
 * Corner pathway.
 * Dipakai untuk muter mengelilingi forest tanpa masuk block hijau.
 */
#define R1_TOP_LEFT_X_BASE                  R1_SAFE_X_LEFT_BASE
#define R1_TOP_LEFT_Y                       R1_SAFE_Y_TOP

#define R1_TOP_RIGHT_X_BASE                 R1_SAFE_X_RIGHT_BASE
#define R1_TOP_RIGHT_Y                      R1_SAFE_Y_TOP

#define R1_BOTTOM_LEFT_X_BASE               R1_SAFE_X_LEFT_BASE
#define R1_BOTTOM_LEFT_Y                    R1_SAFE_Y_BOTTOM

#define R1_BOTTOM_RIGHT_X_BASE              R1_SAFE_X_RIGHT_BASE
#define R1_BOTTOM_RIGHT_Y                   R1_SAFE_Y_BOTTOM

/*
 * Heading BASE BLUE.
 */
#define R1_HEAD_TOP_BASE                      0.0f
#define R1_HEAD_LEFT_BASE                    90.0f
#define R1_HEAD_RIGHT_BASE                  -90.0f
#define R1_HEAD_BOTTOM_BASE                 180.0f

/*
 * Toleransi deteksi posisi robot terhadap pathway.
 * Kalau robot sudah dekat pathway tapi masih terdeteksi OUTSIDE,
 * besarkan nilai ini.
 */
/*
 * Margin dibuat 600 mm supaya:
 * - Posisi start zero (0,0) tetap terdeteksi OUTSIDE, bukan TOP.
 * - Titik entry TOP_LEFT/TOP_RIGHT tetap masuk pathway meskipun safe line
 *   sudah memakai footprint base terbaru.
 */
#define R1_SIDE_DETECT_MARGIN_MM             600.0f

/*
 * Optional clear staff rack.
 * Tidak dipakai di generator.
 * Pakai di sequence sebelum generate path jika robot perlu clear staff rack.
 */
#define R1_CLEAR_STAFF_X_BASE                  0.0f
#define R1_CLEAR_STAFF_Y                     500.0f
#define R1_CLEAR_STAFF_H_BASE                  0.0f

/*
 * ============================================================
 * TYPE
 * ============================================================
 */

typedef enum {
	R1_ARENA_BLUE = 0, R1_ARENA_RED = 1
} R1ArenaMode_t;

typedef enum {
	R1_SIDE_INVALID = 0,
	R1_SIDE_TOP,
	R1_SIDE_RIGHT,
	R1_SIDE_BOTTOM,
	R1_SIDE_LEFT,
	R1_SIDE_OUTSIDE
} R1Side_t;

typedef enum {
	R1_HEADING_PUTAR_DI_AKHIR = 0,

	/*
	 * Putar dulu agar depan robot menghadap forest sesuai sisi path,
	 * lalu jalan sambil heading tetap menghadap forest.
	 */
	R1_HEADING_HADAP_FOREST_SEBELUM_JALAN,

	/*
	 * Putar dulu ke arah waypoint, lalu jalan.
	 * Cocok kalau robot lebih stabil jalan maju.
	 */
	R1_HEADING_HADAP_ARAH_TARGET_SEBELUM_JALAN,

	/*
	 * Jalan langsung sambil heading mengejar heading target.
	 */
	R1_HEADING_BERGERAK_SAMBIL_PUTAR

} R1HeadingMode_t;

typedef enum {
	/*
	 * Mode normal:
	 * 1,2,3     -> TOP
	 * 10,11,12  -> BOTTOM
	 * 4,7       -> LEFT
	 * 6,9       -> RIGHT
	 */
	R1_APPROACH_NORMAL = 0,

	/*
	 * Mode mengurangi R1 berada di area TOP/BOTTOM dekat R2 Entrance/Exit:
	 * 1,4,7,10  -> LEFT
	 * 2         -> TOP
	 * 3,6,9,12  -> RIGHT
	 * 11        -> BOTTOM
	 */
	R1_APPROACH_AVOID_R2_ZONE

} R1ApproachMode_t;

typedef enum {
	R1_GENERATE_IDLE = 0,
	R1_GENERATE_OK,
	R1_GENERATE_INVALID_TARGET,
	R1_GENERATE_PATH_FULL
} R1GenerateStatus_t;

typedef struct {
	float x;
	float y;
	float h;
	R1Side_t side;
	R1HeadingMode_t heading_mode;
} R1PathPoint_t;

typedef struct {
	float x;
	float y;
	float h;
	R1Side_t side;
	uint8_t valid;
} R1Target_t;

/*
 * ============================================================
 * GLOBAL CONFIG
 * ============================================================
 */

extern volatile R1ArenaMode_t R1_CFG_ARENA_MODE;
extern volatile R1ApproachMode_t R1_CFG_APPROACH_MODE;

/*
 * ============================================================
 * DEBUG GLOBAL
 * ============================================================
 *
 * Dipakai untuk Live Expressions.
 */

extern volatile uint8_t R1_DBG_TargetBox;
extern volatile R1HeadingMode_t R1_DBG_HeadingMode;
extern volatile R1ApproachMode_t R1_DBG_ApproachMode;
extern volatile uint8_t R1_DBG_RequestGenerate;

extern volatile uint8_t R1_DBG_PathCount;
extern volatile R1GenerateStatus_t R1_DBG_GenerateStatus;
extern volatile R1Side_t R1_DBG_CurrentSide;
extern volatile R1Side_t R1_DBG_TargetSide;

extern volatile float R1_DBG_BaseCurrentX;
extern volatile float R1_DBG_BaseCurrentY;

extern R1PathPoint_t R1_DBG_Path[R1_MAX_PATH];

extern volatile float R1_DBG_PathX[R1_MAX_PATH];
extern volatile float R1_DBG_PathY[R1_MAX_PATH];
extern volatile float R1_DBG_PathH[R1_MAX_PATH];
extern volatile uint8_t R1_DBG_PathSide[R1_MAX_PATH];
extern volatile uint8_t R1_DBG_PathHeadingMode[R1_MAX_PATH];

/*
 * ============================================================
 * PUBLIC FUNCTION
 * ============================================================
 */

float R1Path_NormalizeHeading(float angle_deg);

float R1Path_ToBaseX(float arena_x);
float R1Path_ToArenaX(float base_x);
float R1Path_ToArenaH(float base_h);

float R1Path_GetClearStaffX(void);
float R1Path_GetClearStaffY(void);
float R1Path_GetClearStaffH(void);

float R1Path_GetSafeXBySideBase(R1Side_t side);
float R1Path_GetSafeYBySideBase(R1Side_t side);
float R1Path_GetSafeXBySideArena(R1Side_t side);

R1Target_t R1Path_GetMeihuaTarget(uint8_t box);
R1Side_t R1Path_DetectCurrentSide(void);

uint8_t R1Path_GenerateToMeihua(uint8_t target_box, R1HeadingMode_t mode,
		R1PathPoint_t path[], uint8_t max_path);

void R1Path_ClearPath(R1PathPoint_t path[], uint8_t max_path);

/*
 * Panggil berkala di task saat debug.
 * Jika R1_DBG_RequestGenerate = 1, fungsi akan generate ulang path.
 */
void R1Path_DebugGenerateTask(void);

#ifdef __cplusplus
}
#endif

#endif /* R1_PATH_PLANNER_H */
