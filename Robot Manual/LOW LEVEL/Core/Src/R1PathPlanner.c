#include "R1PathPlanner.h"

/*
 * ============================================================
 * EXTERNAL POSITION FROM ROBOT ODOMETRY / ABSOLUTE POSITION
 * ============================================================
 *
 * Pastikan nama global ini sama dengan program utama kamu.
 * SetWorldPosition() tetap dilakukan di luar library ini.
 */
extern float PosX_World;
extern float PosY_World;
extern float RobotYaw;

/*
 * ============================================================
 * GLOBAL CONFIG
 * ============================================================
 */

volatile R1ArenaMode_t R1_CFG_ARENA_MODE = R1_ARENA_BLUE;
volatile R1ApproachMode_t R1_CFG_APPROACH_MODE = R1_APPROACH_NORMAL;

/*
 * ============================================================
 * DEBUG GLOBAL
 * ============================================================
 */

volatile uint8_t R1_DBG_TargetBox = 0;
volatile R1HeadingMode_t R1_DBG_HeadingMode =
		R1_HEADING_HADAP_FOREST_SEBELUM_JALAN;
volatile R1ApproachMode_t R1_DBG_ApproachMode = R1_APPROACH_NORMAL;
volatile uint8_t R1_DBG_RequestGenerate = 0;

volatile uint8_t R1_DBG_PathCount = 0;
volatile R1GenerateStatus_t R1_DBG_GenerateStatus = R1_GENERATE_IDLE;
volatile R1Side_t R1_DBG_CurrentSide = R1_SIDE_INVALID;
volatile R1Side_t R1_DBG_TargetSide = R1_SIDE_INVALID;

volatile float R1_DBG_BaseCurrentX = 0.0f;
volatile float R1_DBG_BaseCurrentY = 0.0f;

R1PathPoint_t R1_DBG_Path[R1_MAX_PATH];

volatile float R1_DBG_PathX[R1_MAX_PATH];
volatile float R1_DBG_PathY[R1_MAX_PATH];
volatile float R1_DBG_PathH[R1_MAX_PATH];
volatile uint8_t R1_DBG_PathSide[R1_MAX_PATH];
volatile uint8_t R1_DBG_PathHeadingMode[R1_MAX_PATH];

/*
 * ============================================================
 * BASIC HELPER
 * ============================================================
 */

float R1Path_NormalizeHeading(float angle_deg) {
	while (angle_deg > 180.0f) {
		angle_deg -= 360.0f;
	}

	while (angle_deg < -180.0f) {
		angle_deg += 360.0f;
	}

	return angle_deg;
}

/*
 * Semua planning dilakukan di BASE BLUE.
 *
 * BLUE:
 *   arena_x = base_x
 *
 * RED:
 *   arena_x = -base_x
 */
float R1Path_ToBaseX(float arena_x) {
	if (R1_CFG_ARENA_MODE == R1_ARENA_RED) {
		return -arena_x;
	}

	return arena_x;
}

float R1Path_ToArenaX(float base_x) {
	if (R1_CFG_ARENA_MODE == R1_ARENA_RED) {
		return -base_x;
	}

	return base_x;
}

float R1Path_ToArenaH(float base_h) {
	if (R1_CFG_ARENA_MODE == R1_ARENA_RED) {
		return R1Path_NormalizeHeading(-base_h);
	}

	return R1Path_NormalizeHeading(base_h);
}

float R1Path_GetClearStaffX(void) {
	return R1Path_ToArenaX(R1_CLEAR_STAFF_X_BASE);
}

float R1Path_GetClearStaffY(void) {
	return R1_CLEAR_STAFF_Y;
}

float R1Path_GetClearStaffH(void) {
	return R1Path_ToArenaH(R1_CLEAR_STAFF_H_BASE);
}

/*
 * Safe coordinate berdasarkan FINAL side robot.
 * Fungsi ini sengaja dibuat eksplisit supaya target/corner tidak menghitung
 * offset manual di banyak tempat.
 *
 * Catatan patch LEFT centered:
 * - Untuk R1_SIDE_LEFT, nilai R1_SAFE_X_LEFT_BASE bukan lagi forest-left - offset.
 * - Nilai itu sudah dihitung dari tembok kiri dan sisi kiri forest, lalu base
 *   di-center-kan berdasarkan panjang real base FRONT+REAR.
 */
float R1Path_GetSafeXBySideBase(R1Side_t side) {
	if (side == R1_SIDE_LEFT) {
		return R1_SAFE_X_LEFT_BASE;
	}

	if (side == R1_SIDE_RIGHT) {
		return R1_SAFE_X_RIGHT_BASE;
	}

	return 0.0f;
}

float R1Path_GetSafeYBySideBase(R1Side_t side) {
	if (side == R1_SIDE_TOP) {
		return R1_SAFE_Y_TOP;
	}

	if (side == R1_SIDE_BOTTOM) {
		return R1_SAFE_Y_BOTTOM;
	}

	return 0.0f;
}

float R1Path_GetSafeXBySideArena(R1Side_t side) {
	return R1Path_ToArenaX(R1Path_GetSafeXBySideBase(side));
}

static void R1Path_CopyToDebugArray(void) {
	uint8_t i;

	for (i = 0; i < R1_MAX_PATH; i++) {
		R1_DBG_PathX[i] = 0.0f;
		R1_DBG_PathY[i] = 0.0f;
		R1_DBG_PathH[i] = 0.0f;
		R1_DBG_PathSide[i] = (uint8_t) R1_SIDE_INVALID;
		R1_DBG_PathHeadingMode[i] = (uint8_t) R1_HEADING_PUTAR_DI_AKHIR;
	}

	for (i = 0; i < R1_DBG_PathCount; i++) {
		R1_DBG_PathX[i] = R1_DBG_Path[i].x;
		R1_DBG_PathY[i] = R1_DBG_Path[i].y;
		R1_DBG_PathH[i] = R1_DBG_Path[i].h;
		R1_DBG_PathSide[i] = (uint8_t) R1_DBG_Path[i].side;
		R1_DBG_PathHeadingMode[i] = (uint8_t) R1_DBG_Path[i].heading_mode;
	}
}

static float R1Path_AbsFloat(float value) {
	if (value < 0.0f) {
		return -value;
	}

	return value;
}

static uint8_t R1Path_IsSamePoint(float x1, float y1, float h1, float x2,
		float y2, float h2) {
	float dx = R1Path_AbsFloat(x1 - x2);
	float dy = R1Path_AbsFloat(y1 - y2);
	float dh = R1Path_AbsFloat(R1Path_NormalizeHeading(h1 - h2));

	if (dx < 1.0f && dy < 1.0f && dh < 1.0f) {
		return 1u;
	}

	return 0u;
}

void R1Path_ClearPath(R1PathPoint_t path[], uint8_t max_path) {
	uint8_t i;

	if (path == 0) {
		return;
	}

	for (i = 0; i < max_path; i++) {
		path[i].x = 0.0f;
		path[i].y = 0.0f;
		path[i].h = 0.0f;
		path[i].side = R1_SIDE_INVALID;
		path[i].heading_mode = R1_HEADING_PUTAR_DI_AKHIR;
	}
}

/*
 * Tambah path dari koordinat BASE BLUE.
 * Di sini baru diubah menjadi koordinat arena RED/BLUE aktual.
 */
static uint8_t R1Path_AddBasePath(R1PathPoint_t path[], uint8_t max_path,
		uint8_t *count, float base_x, float y, float base_h, R1Side_t side,
		R1HeadingMode_t mode) {
	if (path == 0 || count == 0) {
		return 0;
	}

	if (*count >= max_path) {
		R1_DBG_GenerateStatus = R1_GENERATE_PATH_FULL;
		return 0;
	}

	{
		float arena_x = R1Path_ToArenaX(base_x);
		float arena_h = R1Path_ToArenaH(base_h);

		if (*count > 0u) {
			if (R1Path_IsSamePoint(path[(*count) - 1u].x, path[(*count) - 1u].y,
					path[(*count) - 1u].h, arena_x, y, arena_h) == 1u) {
				return 1u;
			}
		}

		path[*count].x = arena_x;
		path[*count].y = y;
		path[*count].h = arena_h;
		path[*count].side = side;
		path[*count].heading_mode = mode;
	}

	(*count)++;

	return 1u;
}

/*
 * Tambah path yang target-nya sudah dalam koordinat arena aktual.
 */
static uint8_t R1Path_AddArenaPath(R1PathPoint_t path[], uint8_t max_path,
		uint8_t *count, float arena_x, float y, float arena_h, R1Side_t side,
		R1HeadingMode_t mode) {
	if (path == 0 || count == 0) {
		return 0;
	}

	if (*count >= max_path) {
		R1_DBG_GenerateStatus = R1_GENERATE_PATH_FULL;
		return 0;
	}

	{
		float normalized_h = R1Path_NormalizeHeading(arena_h);

		if (*count > 0u) {
			if (R1Path_IsSamePoint(path[(*count) - 1u].x, path[(*count) - 1u].y,
					path[(*count) - 1u].h, arena_x, y, normalized_h) == 1u) {
				return 1u;
			}
		}

		path[*count].x = arena_x;
		path[*count].y = y;
		path[*count].h = normalized_h;
		path[*count].side = side;
		path[*count].heading_mode = mode;
	}

	(*count)++;

	return 1u;
}

/*
 * ============================================================
 * TARGET MAPPING
 * ============================================================
 *
 * Target yang valid untuk R1:
 * - 1, 2, 3      = TOP
 * - 4, 7         = LEFT
 * - 6, 9         = RIGHT
 * - 10, 11, 12   = BOTTOM
 *
 * Target 5 dan 8 invalid karena berada di tengah forest.
 */
R1Target_t R1Path_GetMeihuaTarget(uint8_t box) {
	R1Target_t target;

	target.x = 0.0f;
	target.y = 0.0f;
	target.h = 0.0f;
	target.side = R1_SIDE_INVALID;
	target.valid = 0;

	if (box < 1u || box > 12u) {
		return target;
	}

	if (box == 5u || box == 8u) {
		return target;
	}

	/*
	 * Layout base:
	 *
	 *  1   2   3
	 *  4   5   6
	 *  7   8   9
	 * 10  11  12
	 */
	{
		uint8_t index = box - 1u;
		uint8_t row = index / 3u;
		uint8_t col = index % 3u;

		float box_center_x_base = R1_FOREST_LEFT_X_BASE
				+ ((float) col * R1_BLOCK_SIZE_MM) + (R1_BLOCK_SIZE_MM / 2.0f);
		float box_center_y = R1_FOREST_TOP_Y + ((float) row * R1_BLOCK_SIZE_MM)
				+ (R1_BLOCK_SIZE_MM / 2.0f);

		/*
		 * =====================================================
		 * MODE NORMAL
		 * =====================================================
		 * 1,2,3      -> TOP
		 * 4,7        -> LEFT
		 * 6,9        -> RIGHT
		 * 10,11,12   -> BOTTOM
		 */
		if (R1_CFG_APPROACH_MODE == R1_APPROACH_NORMAL) {
			if (box == 1u || box == 2u || box == 3u) {
				target.x = R1Path_ToArenaX(box_center_x_base);
				target.y = R1Path_GetSafeYBySideBase(R1_SIDE_TOP);
				target.h = R1Path_ToArenaH(R1_HEAD_TOP_BASE);
				target.side = R1_SIDE_TOP;
				target.valid = 1u;
			} else if (box == 4u || box == 7u) {
				target.x = R1Path_GetSafeXBySideArena(R1_SIDE_LEFT);
				target.y = box_center_y;
				target.h = R1Path_ToArenaH(R1_HEAD_LEFT_BASE);
				target.side = R1_SIDE_LEFT;
				target.valid = 1u;
			} else if (box == 6u || box == 9u) {
				target.x = R1Path_GetSafeXBySideArena(R1_SIDE_RIGHT);
				target.y = box_center_y;
				target.h = R1Path_ToArenaH(R1_HEAD_RIGHT_BASE);
				target.side = R1_SIDE_RIGHT;
				target.valid = 1u;
			} else if (box == 10u || box == 11u || box == 12u) {
				target.x = R1Path_ToArenaX(box_center_x_base);
				target.y = R1Path_GetSafeYBySideBase(R1_SIDE_BOTTOM);
				target.h = R1Path_ToArenaH(R1_HEAD_BOTTOM_BASE);
				target.side = R1_SIDE_BOTTOM;
				target.valid = 1u;
			}

			return target;
		}

		/*
		 * =====================================================
		 * MODE AVOID R2 ZONE
		 * =====================================================
		 * Tujuan:
		 * - TOP hanya dipakai untuk MF 2.
		 * - BOTTOM hanya dipakai untuk MF 11.
		 * - Box pojok / kolom kiri diambil dari LEFT.
		 * - Box pojok / kolom kanan diambil dari RIGHT.
		 *
		 * Mapping:
		 * 1,4,7,10   -> LEFT
		 * 2          -> TOP
		 * 3,6,9,12   -> RIGHT
		 * 11         -> BOTTOM
		 */
		if (R1_CFG_APPROACH_MODE == R1_APPROACH_AVOID_R2_ZONE) {
			if (box == 2u) {
				target.x = R1Path_ToArenaX(box_center_x_base);
				target.y = R1Path_GetSafeYBySideBase(R1_SIDE_TOP);
				target.h = R1Path_ToArenaH(R1_HEAD_TOP_BASE);
				target.side = R1_SIDE_TOP;
				target.valid = 1u;
			} else if (box == 11u) {
				target.x = R1Path_ToArenaX(box_center_x_base);
				target.y = R1Path_GetSafeYBySideBase(R1_SIDE_BOTTOM);
				target.h = R1Path_ToArenaH(R1_HEAD_BOTTOM_BASE);
				target.side = R1_SIDE_BOTTOM;
				target.valid = 1u;
			} else if (box == 1u || box == 4u || box == 7u || box == 10u) {
				target.x = R1Path_GetSafeXBySideArena(R1_SIDE_LEFT);
				target.y = box_center_y;
				target.h = R1Path_ToArenaH(R1_HEAD_LEFT_BASE);
				target.side = R1_SIDE_LEFT;
				target.valid = 1u;
			} else if (box == 3u || box == 6u || box == 9u || box == 12u) {
				target.x = R1Path_GetSafeXBySideArena(R1_SIDE_RIGHT);
				target.y = box_center_y;
				target.h = R1Path_ToArenaH(R1_HEAD_RIGHT_BASE);
				target.side = R1_SIDE_RIGHT;
				target.valid = 1u;
			}

			return target;
		}
	}

	return target;
}

/*
 * ============================================================
 * CURRENT SIDE DETECTION
 * ============================================================
 *
 * Deteksi dilakukan di koordinat BASE BLUE.
 * Untuk RED, PosX_World di-mirror dulu.
 */
R1Side_t R1Path_DetectCurrentSide(void) {
	float base_x = R1Path_ToBaseX(PosX_World);
	float y = PosY_World;

	float margin = R1_SIDE_DETECT_MARGIN_MM;

	R1_DBG_BaseCurrentX = base_x;
	R1_DBG_BaseCurrentY = y;

	/*
	 * TOP pathway.
	 */
	if ((y < R1_FOREST_TOP_Y) && (base_x >= (R1_FOREST_LEFT_X_BASE - margin))
			&& (base_x <= (R1_FOREST_RIGHT_X_BASE + margin))) {
		return R1_SIDE_TOP;
	}

	/*
	 * BOTTOM pathway.
	 */
	if ((y > R1_FOREST_BOTTOM_Y) && (base_x >= (R1_FOREST_LEFT_X_BASE - margin))
			&& (base_x <= (R1_FOREST_RIGHT_X_BASE + margin))) {
		return R1_SIDE_BOTTOM;
	}

	/*
	 * LEFT pathway.
	 */
	if ((base_x < R1_FOREST_LEFT_X_BASE) && (y >= (R1_FOREST_TOP_Y - margin))
			&& (y <= (R1_FOREST_BOTTOM_Y + margin))) {
		return R1_SIDE_LEFT;
	}

	/*
	 * RIGHT pathway.
	 */
	if ((base_x > R1_FOREST_RIGHT_X_BASE) && (y >= (R1_FOREST_TOP_Y - margin))
			&& (y <= (R1_FOREST_BOTTOM_Y + margin))) {
		return R1_SIDE_RIGHT;
	}

	/*
	 * Start Zone, Martial Club, Staff Rack, atau area transisi.
	 */
	return R1_SIDE_OUTSIDE;
}

/*
 * ============================================================
 * PATH GENERATOR
 * ============================================================
 *
 * v3 behavior:
 * - Kalau current_side == OUTSIDE dan target side TOP:
 *      langsung ke target box.
 *
 * - Kalau current_side == OUTSIDE dan target bukan TOP:
 *      masuk dari corner TOP yang paling masuk akal,
 *      bukan selalu entry tengah.
 */
uint8_t R1Path_GenerateToMeihua(uint8_t target_box, R1HeadingMode_t mode,
		R1PathPoint_t path[], uint8_t max_path) {
	uint8_t count = 0u;
	R1Target_t target;
	R1Side_t current_side;

	R1_DBG_GenerateStatus = R1_GENERATE_IDLE;

	R1Path_ClearPath(path, max_path);

	target = R1Path_GetMeihuaTarget(target_box);
	current_side = R1Path_DetectCurrentSide();

	R1_DBG_CurrentSide = current_side;
	R1_DBG_TargetSide = target.side;

	if (target.valid == 0u) {
		R1_DBG_GenerateStatus = R1_GENERATE_INVALID_TARGET;
		return 0u;
	}

	/*
	 * ========================================================
	 * OUTSIDE HANDLING
	 * ========================================================
	 *
	 * R1 tidak punya entrance zone khusus.
	 * Maka dari Start Zone / MC boleh langsung masuk ke depan target
	 * jika target berada di sisi TOP.
	 *
	 * Untuk target LEFT / RIGHT / BOTTOM, masuk lewat corner TOP
	 * yang sesuai.
	 */
	if (current_side == R1_SIDE_OUTSIDE) {
		if (target.side == R1_SIDE_TOP) {
			R1Path_AddArenaPath(path, max_path, &count, target.x, target.y,
					target.h, target.side, mode);

			R1_DBG_GenerateStatus = R1_GENERATE_OK;
			return count;
		}

		if (target.side == R1_SIDE_LEFT) {
			R1Path_AddBasePath(path, max_path, &count,
			R1_TOP_LEFT_X_BASE,
			R1_TOP_LEFT_Y,
			R1_HEAD_TOP_BASE, R1_SIDE_TOP, mode);

			current_side = R1_SIDE_TOP;
		} else if (target.side == R1_SIDE_RIGHT) {
			R1Path_AddBasePath(path, max_path, &count,
			R1_TOP_RIGHT_X_BASE,
			R1_TOP_RIGHT_Y,
			R1_HEAD_TOP_BASE, R1_SIDE_TOP, mode);

			current_side = R1_SIDE_TOP;
		} else if (target.side == R1_SIDE_BOTTOM) {
			/*
			 * Box 10/11 lewat kiri, box 12 lewat kanan.
			 * Ini bisa dituning lagi kalau strategi berubah.
			 */
			if (target_box == 10u || target_box == 11u) {
				R1Path_AddBasePath(path, max_path, &count,
				R1_TOP_LEFT_X_BASE,
				R1_TOP_LEFT_Y,
				R1_HEAD_TOP_BASE, R1_SIDE_TOP, mode);

				current_side = R1_SIDE_TOP;
			} else {
				R1Path_AddBasePath(path, max_path, &count,
				R1_TOP_RIGHT_X_BASE,
				R1_TOP_RIGHT_Y,
				R1_HEAD_TOP_BASE, R1_SIDE_TOP, mode);

				current_side = R1_SIDE_TOP;
			}
		}
	}

	/*
	 * Kalau setelah OUTSIDE handling ternyata sudah satu sisi,
	 * langsung menuju target.
	 */
	if (current_side == target.side) {
		R1Path_AddArenaPath(path, max_path, &count, target.x, target.y,
				target.h, target.side, mode);

		R1_DBG_GenerateStatus = R1_GENERATE_OK;
		return count;
	}

	/*
	 * ========================================================
	 * ROUTING DARI TOP
	 * ========================================================
	 */
	if (current_side == R1_SIDE_TOP) {
		if (target.side == R1_SIDE_LEFT) {
			R1Path_AddBasePath(path, max_path, &count,
			R1_TOP_LEFT_X_BASE, R1_TOP_LEFT_Y,
			R1_HEAD_TOP_BASE, R1_SIDE_TOP, mode);
		} else if (target.side == R1_SIDE_RIGHT) {
			R1Path_AddBasePath(path, max_path, &count,
			R1_TOP_RIGHT_X_BASE, R1_TOP_RIGHT_Y,
			R1_HEAD_TOP_BASE, R1_SIDE_TOP, mode);
		} else if (target.side == R1_SIDE_BOTTOM) {
			if (target_box == 10u || target_box == 11u) {
				R1Path_AddBasePath(path, max_path, &count,
				R1_TOP_LEFT_X_BASE, R1_TOP_LEFT_Y,
				R1_HEAD_TOP_BASE, R1_SIDE_TOP, mode);

				R1Path_AddBasePath(path, max_path, &count,
				R1_BOTTOM_LEFT_X_BASE, R1_BOTTOM_LEFT_Y,
				R1_HEAD_LEFT_BASE, R1_SIDE_LEFT, mode);
			} else {
				R1Path_AddBasePath(path, max_path, &count,
				R1_TOP_RIGHT_X_BASE, R1_TOP_RIGHT_Y,
				R1_HEAD_TOP_BASE, R1_SIDE_TOP, mode);

				R1Path_AddBasePath(path, max_path, &count,
				R1_BOTTOM_RIGHT_X_BASE, R1_BOTTOM_RIGHT_Y,
				R1_HEAD_RIGHT_BASE, R1_SIDE_RIGHT, mode);
			}
		}
	}

	/*
	 * ========================================================
	 * ROUTING DARI LEFT
	 * ========================================================
	 */
	else if (current_side == R1_SIDE_LEFT) {
		if (target.side == R1_SIDE_TOP) {
			R1Path_AddBasePath(path, max_path, &count,
			R1_TOP_LEFT_X_BASE, R1_TOP_LEFT_Y,
			R1_HEAD_LEFT_BASE, R1_SIDE_LEFT, mode);
		} else if (target.side == R1_SIDE_BOTTOM) {
			R1Path_AddBasePath(path, max_path, &count,
			R1_BOTTOM_LEFT_X_BASE, R1_BOTTOM_LEFT_Y,
			R1_HEAD_LEFT_BASE, R1_SIDE_LEFT, mode);
		} else if (target.side == R1_SIDE_RIGHT) {
			R1Path_AddBasePath(path, max_path, &count,
			R1_BOTTOM_LEFT_X_BASE, R1_BOTTOM_LEFT_Y,
			R1_HEAD_LEFT_BASE, R1_SIDE_LEFT, mode);

			R1Path_AddBasePath(path, max_path, &count,
			R1_BOTTOM_RIGHT_X_BASE, R1_BOTTOM_RIGHT_Y,
			R1_HEAD_BOTTOM_BASE, R1_SIDE_BOTTOM, mode);
		}
	}

	/*
	 * ========================================================
	 * ROUTING DARI RIGHT
	 * ========================================================
	 */
	else if (current_side == R1_SIDE_RIGHT) {
		if (target.side == R1_SIDE_TOP) {
			R1Path_AddBasePath(path, max_path, &count,
			R1_TOP_RIGHT_X_BASE, R1_TOP_RIGHT_Y,
			R1_HEAD_RIGHT_BASE, R1_SIDE_RIGHT, mode);
		} else if (target.side == R1_SIDE_BOTTOM) {
			R1Path_AddBasePath(path, max_path, &count,
			R1_BOTTOM_RIGHT_X_BASE, R1_BOTTOM_RIGHT_Y,
			R1_HEAD_RIGHT_BASE, R1_SIDE_RIGHT, mode);
		} else if (target.side == R1_SIDE_LEFT) {
			R1Path_AddBasePath(path, max_path, &count,
			R1_BOTTOM_RIGHT_X_BASE, R1_BOTTOM_RIGHT_Y,
			R1_HEAD_RIGHT_BASE, R1_SIDE_RIGHT, mode);

			R1Path_AddBasePath(path, max_path, &count,
			R1_BOTTOM_LEFT_X_BASE, R1_BOTTOM_LEFT_Y,
			R1_HEAD_BOTTOM_BASE, R1_SIDE_BOTTOM, mode);
		}
	}

	/*
	 * ========================================================
	 * ROUTING DARI BOTTOM
	 * ========================================================
	 */
	else if (current_side == R1_SIDE_BOTTOM) {
		if (target.side == R1_SIDE_LEFT) {
			R1Path_AddBasePath(path, max_path, &count,
			R1_BOTTOM_LEFT_X_BASE, R1_BOTTOM_LEFT_Y,
			R1_HEAD_BOTTOM_BASE, R1_SIDE_BOTTOM, mode);
		} else if (target.side == R1_SIDE_RIGHT) {
			R1Path_AddBasePath(path, max_path, &count,
			R1_BOTTOM_RIGHT_X_BASE, R1_BOTTOM_RIGHT_Y,
			R1_HEAD_BOTTOM_BASE, R1_SIDE_BOTTOM, mode);
		} else if (target.side == R1_SIDE_TOP) {
			if (target_box == 1u || target_box == 2u) {
				R1Path_AddBasePath(path, max_path, &count,
				R1_BOTTOM_LEFT_X_BASE, R1_BOTTOM_LEFT_Y,
				R1_HEAD_BOTTOM_BASE, R1_SIDE_BOTTOM, mode);

				R1Path_AddBasePath(path, max_path, &count,
				R1_TOP_LEFT_X_BASE, R1_TOP_LEFT_Y,
				R1_HEAD_LEFT_BASE, R1_SIDE_LEFT, mode);
			} else {
				R1Path_AddBasePath(path, max_path, &count,
				R1_BOTTOM_RIGHT_X_BASE, R1_BOTTOM_RIGHT_Y,
				R1_HEAD_BOTTOM_BASE, R1_SIDE_BOTTOM, mode);

				R1Path_AddBasePath(path, max_path, &count,
				R1_TOP_RIGHT_X_BASE, R1_TOP_RIGHT_Y,
				R1_HEAD_RIGHT_BASE, R1_SIDE_RIGHT, mode);
			}
		}
	}

	/*
	 * Target terakhir selalu depan box.
	 */
	R1Path_AddArenaPath(path, max_path, &count, target.x, target.y, target.h,
			target.side, mode);

	if (R1_DBG_GenerateStatus != R1_GENERATE_PATH_FULL) {
		R1_DBG_GenerateStatus = R1_GENERATE_OK;
	}

	return count;
}

/*
 * ============================================================
 * DEBUG TASK
 * ============================================================
 */

void R1Path_DebugGenerateTask(void) {
	if (R1_DBG_RequestGenerate == 0u) {
		return;
	}

	R1_DBG_RequestGenerate = 0u;

	/*
	 * Debug helper:
	 * Supaya di Live Expressions cukup mengubah R1_DBG_ApproachMode,
	 * lalu request generate.
	 */
	R1_CFG_APPROACH_MODE = R1_DBG_ApproachMode;

	R1_DBG_PathCount = R1Path_GenerateToMeihua((uint8_t) R1_DBG_TargetBox,
			(R1HeadingMode_t) R1_DBG_HeadingMode, R1_DBG_Path,
			R1_MAX_PATH);

	R1Path_CopyToDebugArray();
}
