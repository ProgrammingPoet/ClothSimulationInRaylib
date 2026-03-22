#include <raylib.h>
#include <iostream>
#include <array>
#include <cmath>
#include <chrono>
#include <thread>
#include <queue>
#include <string>
#include <semaphore>
#include <vector>
#include <atomic>
#include <immintrin.h>



constexpr int SCREENX = 1280;
constexpr int SCREENY = 900;
constexpr int SIDE = 200;
constexpr int COLUMNS = SIDE;
constexpr int ROWS = SIDE;
constexpr int WIDTH = 800;
constexpr int HEIGHT = 500;
constexpr float g = 2000;
constexpr int RADIUS = 10;
constexpr float Damping = 0.98;
constexpr int PASSES = 60;
constexpr int FIBERLENGTH = 10;


constexpr int NUMBEROFPOINTS = ROWS * COLUMNS;
constexpr int RADIUSSQUARED = RADIUS * RADIUS;
constexpr int SPACINGX = WIDTH / (float)(COLUMNS - 1);
constexpr int SPACINGY = HEIGHT / (float)(ROWS - 1);
constexpr int LENGTHX = SPACINGX;
constexpr int LENGTHY = SPACINGY;
constexpr float STARTX = (SCREENX - WIDTH) / 2.0f;
constexpr float STARTY = (SCREENY - HEIGHT) / 2.0f;

alignas(32) std::array<float, NUMBEROFPOINTS> CURRENTPOSITIONSX;
alignas(32) std::array<float, NUMBEROFPOINTS> CURRENTPOSITIONSY;
alignas(32) std::array<float, NUMBEROFPOINTS> PREVIOUSPOSITIONSX;
alignas(32) std::array<float, NUMBEROFPOINTS> PREVIOUSPOSITIONSY;
alignas(32) std::array<float, NUMBEROFPOINTS> CURRENTPOSITIONSXT;
alignas(32) std::array<float, NUMBEROFPOINTS> CURRENTPOSITIONSYT;
alignas(32) std::array<float, NUMBEROFPOINTS> PREVIOUSPOSITIONSXT;
alignas(32) std::array<float, NUMBEROFPOINTS> PREVIOUSPOSITIONSYT;
alignas(32) std::array<float, ROWS* COLUMNS> VERTICALSTICKSCONNECTED;
alignas(32) std::array<float, ROWS* COLUMNS> HORIZONTALSTICKSCONNECTED;
alignas(32) std::array<float, ROWS* COLUMNS> VERTICALSTICKSCONNECTEDT;
alignas(32) std::array<float, ROWS* COLUMNS> HORIZONTALSTICKSCONNECTEDT;


std::chrono::steady_clock::time_point CURRENTTIME = std::chrono::steady_clock::now();
std::chrono::steady_clock::time_point PREVIOUSTIME;
float DELTATIME;



void InitializePointsAndSticks() {

	for (int ROW = 0; ROW < ROWS; ROW++) {
		for (int COLUMN = 0; COLUMN < COLUMNS; COLUMN++) {
			int POINT = COLUMN + ROW * COLUMNS;
			PREVIOUSPOSITIONSX[POINT] = STARTX + COLUMN * SPACINGX;
			PREVIOUSPOSITIONSY[POINT] = STARTY + ROW * SPACINGY;
			CURRENTPOSITIONSX[POINT] = PREVIOUSPOSITIONSX[POINT];
			CURRENTPOSITIONSY[POINT] = PREVIOUSPOSITIONSY[POINT];
		}
	}
	for (int ROW = 0; ROW < ROWS; ROW++) {
		for (int COLUMN = 0; COLUMN < COLUMNS; COLUMN++) {
			int POINT = COLUMN + ROW * COLUMNS;
			if (COLUMN == COLUMNS - 1) {
				HORIZONTALSTICKSCONNECTED[POINT] = 0;
			}
			else {
				HORIZONTALSTICKSCONNECTED[POINT] = 1;
			}
			if (ROW == ROWS - 1) {
				VERTICALSTICKSCONNECTED[POINT] = 0;
			}
			else {
				VERTICALSTICKSCONNECTED[POINT] = 1;

			}
		}
	}
}

int NextVerticalPoint(int POINT) {
	if (POINT >= COLUMNS * (ROWS - 1) && POINT < COLUMNS * ROWS - 1) {
		POINT = POINT % COLUMNS + 1;
		return POINT;
	}
	else if (POINT < COLUMNS * (ROWS - 1)) {
		return POINT + COLUMNS;
	}
	else if (POINT == COLUMNS * ROWS - 1) {
		return COLUMNS * ROWS;
	}
}
int NextHorizontalPoint(int POINT) {
	return POINT + 1;
}

void TransposeArrays() {
	for (int ROW = 0; ROW < ROWS; ROW++) {
		for (int COLUMN = 0; COLUMN < COLUMNS; COLUMN++) {
			CURRENTPOSITIONSXT[ROW + COLUMN * ROWS] = CURRENTPOSITIONSX[COLUMN + ROW * COLUMNS];
			CURRENTPOSITIONSYT[ROW + COLUMN * ROWS] = CURRENTPOSITIONSY[COLUMN + ROW * COLUMNS];
			PREVIOUSPOSITIONSXT[ROW + COLUMN * ROWS] = PREVIOUSPOSITIONSX[COLUMN + ROW * COLUMNS];
			PREVIOUSPOSITIONSYT[ROW + COLUMN * ROWS] = PREVIOUSPOSITIONSY[COLUMN + ROW * COLUMNS];
			VERTICALSTICKSCONNECTEDT[ROW + COLUMN * ROWS] = VERTICALSTICKSCONNECTED[COLUMN + ROW * COLUMNS];
			HORIZONTALSTICKSCONNECTEDT[ROW + COLUMN * ROWS] = HORIZONTALSTICKSCONNECTED[COLUMN + ROW * COLUMNS];
		}
	}
}

void CopyBackTransposedArrays() {
	for (int ROW = 0; ROW < ROWS; ROW++) {
		for (int COLUMN = 0; COLUMN < COLUMNS; COLUMN++) {
			CURRENTPOSITIONSX[COLUMN + ROW * COLUMNS] = CURRENTPOSITIONSXT[ROW + COLUMN * ROWS];
			CURRENTPOSITIONSY[COLUMN + ROW * COLUMNS] = CURRENTPOSITIONSYT[ROW + COLUMN * ROWS];
			PREVIOUSPOSITIONSX[COLUMN + ROW * COLUMNS] = PREVIOUSPOSITIONSXT[ROW + COLUMN * ROWS];
			PREVIOUSPOSITIONSY[COLUMN + ROW * COLUMNS] = PREVIOUSPOSITIONSYT[ROW + COLUMN * ROWS];
			VERTICALSTICKSCONNECTED[COLUMN + ROW * COLUMNS] = VERTICALSTICKSCONNECTEDT[ROW + COLUMN * ROWS];
			HORIZONTALSTICKSCONNECTED[COLUMN + ROW * COLUMNS] = HORIZONTALSTICKSCONNECTEDT[ROW + COLUMN * ROWS];
		}
	}
}

void ApplyGravityToPoints() {
	float Gravity = g * DELTATIME * DELTATIME;
	for (int POINT = COLUMNS; POINT < NUMBEROFPOINTS; POINT += 8) {
		__m256 OLDPOSTIONSX = _mm256_load_ps(&PREVIOUSPOSITIONSX[POINT]);
		__m256 OLDPOSTIONSY = _mm256_load_ps(&PREVIOUSPOSITIONSY[POINT]);
		__m256 NEWPOSITIONSX = _mm256_load_ps(&CURRENTPOSITIONSX[POINT]);
		__m256 NEWPOSITIONSY = _mm256_load_ps(&CURRENTPOSITIONSY[POINT]);
		__m256 VELOCITYX = _mm256_sub_ps(NEWPOSITIONSX, OLDPOSTIONSX);
		__m256 VELOCITYY = _mm256_sub_ps(NEWPOSITIONSY, OLDPOSTIONSY);
		__m256 GRAVITY = _mm256_set_ps(Gravity, Gravity, Gravity, Gravity, Gravity, Gravity, Gravity, Gravity);
		__m256 TEMPORARYX = NEWPOSITIONSX;
		__m256 TEMPORARYY = NEWPOSITIONSY;
		__m256 DAMPING = _mm256_set_ps(Damping, Damping, Damping, Damping, Damping, Damping, Damping, Damping);
		NEWPOSITIONSX = _mm256_add_ps(_mm256_mul_ps(VELOCITYX, DAMPING), NEWPOSITIONSX);
		NEWPOSITIONSY = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(VELOCITYY, DAMPING), GRAVITY), NEWPOSITIONSY);
		_mm256_store_ps(&CURRENTPOSITIONSX[POINT], NEWPOSITIONSX);
		_mm256_store_ps(&CURRENTPOSITIONSY[POINT], NEWPOSITIONSY);
		_mm256_store_ps(&PREVIOUSPOSITIONSX[POINT], TEMPORARYX);
		_mm256_store_ps(&PREVIOUSPOSITIONSY[POINT], TEMPORARYY);
	}
}

void CutSticks() {
	Vector2 MOUSEPOSITION = GetMousePosition();
	for (int ROW = 0; ROW < ROWS; ROW++) {
		for (int COLUMN = 0; COLUMN < COLUMNS; COLUMN++) {
			int CURRENTPOINT = COLUMN + ROW * COLUMNS;
			int NEXTVERTICALPOINT = NextVerticalPoint(CURRENTPOINT);
			int NEXTHORIZONTALPOINT = NextHorizontalPoint(CURRENTPOINT);
			if (ROW < ROWS - 1) {
				float dx = MOUSEPOSITION.x - (CURRENTPOSITIONSX[CURRENTPOINT] + CURRENTPOSITIONSX[NEXTVERTICALPOINT]) / 2;
				float dy = MOUSEPOSITION.y - (CURRENTPOSITIONSY[CURRENTPOINT] + CURRENTPOSITIONSY[NEXTVERTICALPOINT]) / 2;
				float DISTANSQUARED = dx * dx + dy * dy;
				if (DISTANSQUARED <= RADIUSSQUARED) {
					VERTICALSTICKSCONNECTED[CURRENTPOINT] = 0;
				}
			}
			if (COLUMN < COLUMNS - 1) {
				float dx = MOUSEPOSITION.x - (CURRENTPOSITIONSX[CURRENTPOINT] + CURRENTPOSITIONSX[NextHorizontalPoint(CURRENTPOINT)]) / 2;
				float dy = MOUSEPOSITION.y - (CURRENTPOSITIONSY[CURRENTPOINT] + CURRENTPOSITIONSY[NextHorizontalPoint(CURRENTPOINT)]) / 2;
				float DISTANSQUARED = dx * dx + dy * dy;
				if (DISTANSQUARED <= RADIUSSQUARED) {
					HORIZONTALSTICKSCONNECTED[CURRENTPOINT] = 0;
				}
			}
		}
	}
}

void DrawSticks() {
	for (int ROW = 0; ROW < ROWS; ROW++) {
		for (int COLUMN = 0; COLUMN < COLUMNS; COLUMN++) {
			int CURRENTPOINT = COLUMN + ROW * COLUMNS;
			int NEXTVERTICALPOINT = NextVerticalPoint(CURRENTPOINT);
			int NEXTHORIZONTALPOINT = NextHorizontalPoint(CURRENTPOINT);
			if (VERTICALSTICKSCONNECTED[CURRENTPOINT]) {
				DrawLine(CURRENTPOSITIONSX[CURRENTPOINT], CURRENTPOSITIONSY[CURRENTPOINT], CURRENTPOSITIONSX[NEXTVERTICALPOINT], CURRENTPOSITIONSY[NEXTVERTICALPOINT], WHITE);
			}
			if (HORIZONTALSTICKSCONNECTED[CURRENTPOINT]) {
				DrawLine(CURRENTPOSITIONSX[CURRENTPOINT], CURRENTPOSITIONSY[CURRENTPOINT], CURRENTPOSITIONSX[NEXTHORIZONTALPOINT], CURRENTPOSITIONSY[NEXTHORIZONTALPOINT], WHITE);
			}
		}
	}
}

int THREADS = std::thread::hardware_concurrency();
std::vector<std::thread> THREADPOOL;
std::counting_semaphore<SIDE> WORKSEMAPHORE(0);
std::binary_semaphore DONESEMAPHORE(0);
std::atomic<int> NEXTWORK;
std::atomic<int> WORKDONE;
char WORKALONG;
bool RUNNING = true;


void Pass() {
	NEXTWORK.store(0);
	WORKDONE.store(0);
	WORKALONG = 'X';
	WORKSEMAPHORE.release(ROWS / FIBERLENGTH);
	DONESEMAPHORE.acquire();

	NEXTWORK.store(0);
	WORKDONE.store(0);
	WORKALONG = 'Y';
	WORKSEMAPHORE.release(ROWS / (FIBERLENGTH * 2));
	DONESEMAPHORE.acquire();

	NEXTWORK.store(FIBERLENGTH);
	WORKDONE.store(0);
	WORKALONG = 'Y';
	WORKSEMAPHORE.release(ROWS / (FIBERLENGTH * 2));
	DONESEMAPHORE.acquire();
}

void EvaluateVerticalUsingAVX2(int POINT) {
	int STARTPOINT = POINT;
	if (POINT >= COLUMNS && POINT <= COLUMNS * (ROWS - 1)) {
		for (POINT; POINT < (COLUMNS + STARTPOINT - 1); POINT += 8) {
			__m256 CONNECTED = _mm256_load_ps(&VERTICALSTICKSCONNECTED[POINT]);
			__m256 ROW1X = _mm256_load_ps(&CURRENTPOSITIONSX[POINT]);
			__m256 ROW2X = _mm256_load_ps(&CURRENTPOSITIONSX[NextVerticalPoint(POINT)]);
			__m256 ROW1Y = _mm256_load_ps(&CURRENTPOSITIONSY[POINT]);
			__m256 ROW2Y = _mm256_load_ps(&CURRENTPOSITIONSY[NextVerticalPoint(POINT)]);
			__m256 dx = _mm256_sub_ps(ROW2X, ROW1X);
			__m256 dy = _mm256_sub_ps(ROW2Y, ROW1Y);
			__m256 dx2 = _mm256_mul_ps(dx, dx);
			__m256 dy2 = _mm256_mul_ps(dy, dy);
			__m256 dx2adddy2 = _mm256_add_ps(dx2, dy2);
			__m256 DISTANCE = _mm256_sqrt_ps(dx2adddy2);
			__m256 LENGTH = _mm256_set_ps(LENGTHY, LENGTHY, LENGTHY, LENGTHY, LENGTHY, LENGTHY, LENGTHY, LENGTHY);
			__m256 EXTRALENGTH = _mm256_sub_ps(DISTANCE, LENGTH);
			__m256 CORRECTIONRATIO = _mm256_div_ps(EXTRALENGTH, DISTANCE);
			__m256 CORRECTION = _mm256_div_ps(CORRECTIONRATIO, _mm256_set_ps(2, 2, 2, 2, 2, 2, 2, 2));
			CORRECTION = _mm256_mul_ps(CORRECTION, CONNECTED);
			__m256 dxCORRECTION = _mm256_mul_ps(dx, CORRECTION);
			__m256 dyCORRECTION = _mm256_mul_ps(dy, CORRECTION);
			ROW1X = _mm256_add_ps(ROW1X, dxCORRECTION);
			ROW2X = _mm256_sub_ps(ROW2X, dxCORRECTION);
			ROW1Y = _mm256_add_ps(ROW1Y, dyCORRECTION);
			ROW2Y = _mm256_sub_ps(ROW2Y, dyCORRECTION);

			_mm256_store_ps(&CURRENTPOSITIONSX[POINT], ROW1X);
			_mm256_store_ps(&CURRENTPOSITIONSX[NextVerticalPoint(POINT)], ROW2X);
			_mm256_store_ps(&CURRENTPOSITIONSY[POINT], ROW1Y);
			_mm256_store_ps(&CURRENTPOSITIONSY[NextVerticalPoint(POINT)], ROW2Y);

		}
	}
	else if (POINT < COLUMNS) {
		for (POINT; POINT < (COLUMNS + STARTPOINT - 1); POINT += 8) {
			__m256 CONNECTED = _mm256_load_ps(&VERTICALSTICKSCONNECTED[POINT]);
			__m256 ROW1X = _mm256_load_ps(&CURRENTPOSITIONSX[POINT]);
			__m256 ROW2X = _mm256_load_ps(&CURRENTPOSITIONSX[POINT + COLUMNS]);
			__m256 ROW1Y = _mm256_load_ps(&CURRENTPOSITIONSY[POINT]);
			__m256 ROW2Y = _mm256_load_ps(&CURRENTPOSITIONSY[POINT + COLUMNS]);
			__m256 dx = _mm256_sub_ps(ROW2X, ROW1X);
			__m256 dy = _mm256_sub_ps(ROW2Y, ROW1Y);
			__m256 dx2 = _mm256_mul_ps(dx, dx);
			__m256 dy2 = _mm256_mul_ps(dy, dy);
			__m256 dx2adddy2 = _mm256_add_ps(dx2, dy2);
			__m256 DISTANCE = _mm256_sqrt_ps(dx2adddy2);
			__m256 LENGTH = _mm256_set_ps(LENGTHY, LENGTHY, LENGTHY, LENGTHY, LENGTHY, LENGTHY, LENGTHY, LENGTHY);
			__m256 EXTRALENGTH = _mm256_sub_ps(DISTANCE, LENGTH);
			__m256 CORRECTIONRATIO = _mm256_div_ps(EXTRALENGTH, DISTANCE);
			__m256 CORRECTION = _mm256_mul_ps(CORRECTIONRATIO, CONNECTED);
			__m256 dxCORRECTION = _mm256_mul_ps(dx, CORRECTION);
			__m256 dyCORRECTION = _mm256_mul_ps(dy, CORRECTION);
			ROW2X = _mm256_sub_ps(ROW2X, dxCORRECTION);
			ROW2Y = _mm256_sub_ps(ROW2Y, dyCORRECTION);

			_mm256_store_ps(&CURRENTPOSITIONSX[NextVerticalPoint(POINT)], ROW2X);
			_mm256_store_ps(&CURRENTPOSITIONSY[NextVerticalPoint(POINT)], ROW2Y);
		}
	}
	else if (POINT >= COLUMNS * (ROWS - 1)) {

	}
}


void EvaluateFibers() {
	while (RUNNING) {
		WORKSEMAPHORE.acquire();
		if (WORKALONG == 'X') {
			int INDEX = NEXTWORK.fetch_add(FIBERLENGTH);
			int CURRENTPOINT = INDEX * COLUMNS;
			int LASTPOINT = CURRENTPOINT + FIBERLENGTH * COLUMNS - 1;
			for (CURRENTPOINT; CURRENTPOINT <= LASTPOINT; CURRENTPOINT = NextVerticalPoint(CURRENTPOINT)) {
				int NEXTPOINT = NextHorizontalPoint(CURRENTPOINT);
				int START = CURRENTPOINT;
				if (!HORIZONTALSTICKSCONNECTED[CURRENTPOINT]) {
					continue;
				}
				float dx = CURRENTPOSITIONSX[NEXTPOINT] - CURRENTPOSITIONSX[CURRENTPOINT];
				float dy = CURRENTPOSITIONSY[NEXTPOINT] - CURRENTPOSITIONSY[CURRENTPOINT];
				float DISTANCE = std::sqrt(dx * dx + dy * dy);
				float CORRECTIONRATIO = (DISTANCE - LENGTHX) / DISTANCE;
				float CORRECTION = CORRECTIONRATIO / 2;
				CURRENTPOSITIONSX[CURRENTPOINT] = CURRENTPOSITIONSX[CURRENTPOINT] + dx * CORRECTION;
				CURRENTPOSITIONSY[CURRENTPOINT] = CURRENTPOSITIONSY[CURRENTPOINT] + dy * CORRECTION;
				CURRENTPOSITIONSX[NEXTPOINT] = CURRENTPOSITIONSX[NEXTPOINT] - dx * CORRECTION;
				CURRENTPOSITIONSY[NEXTPOINT] = CURRENTPOSITIONSY[NEXTPOINT] - dy * CORRECTION;
			}
			if (WORKDONE.fetch_add(FIBERLENGTH) == ROWS - FIBERLENGTH) {
				DONESEMAPHORE.release();
			}
		}
		else if (WORKALONG == 'Y') {
			int INDEX = NEXTWORK.fetch_add(2 * FIBERLENGTH);
			int ROW = INDEX;
			int CURRENTPOINT = ROW * COLUMNS;
			for (int ROW = 0; ROW < FIBERLENGTH; ROW++) {
				EvaluateVerticalUsingAVX2(CURRENTPOINT);
				CURRENTPOINT = NextVerticalPoint(CURRENTPOINT);
			}
			int BATCHSIZE = ROWS / (2 * FIBERLENGTH);
			int BATCHESDONE = WORKDONE.fetch_add(1);
			if (BATCHESDONE + 1 == BATCHSIZE) {
				DONESEMAPHORE.release();
			}
		}
	}
}

void InitializeThreads() {
	for (int _ = 0; _ < THREADS - 1; _++) {
		THREADPOOL.emplace_back(std::thread(EvaluateFibers));
	}
}

void Reset() {
	InitializePointsAndSticks();
}




int main() {
	InitializePointsAndSticks();
	InitializeThreads();

	InitWindow(SCREENX, SCREENY, "Cloth Simulation");

	while (!WindowShouldClose()) {
		PREVIOUSTIME = CURRENTTIME;
		CURRENTTIME = std::chrono::steady_clock::now();
		DELTATIME = std::chrono::duration_cast<std::chrono::nanoseconds>(CURRENTTIME - PREVIOUSTIME).count() / (float)1e9;
		BeginDrawing();
		DrawFPS(10, 10);
		ClearBackground(BLACK);

		if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON)) {
			Reset();
		}
		if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
			CutSticks();
		}
		ApplyGravityToPoints();

		for (int PASS = 0; PASS < PASSES; PASS++) {
			Pass();
		}

		DrawSticks();
		EndDrawing();
	}
	CloseWindow();
	RUNNING = false;
	return 0;
}