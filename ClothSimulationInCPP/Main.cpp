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



constexpr int SCREENX = 1280;
constexpr int SCREENY = 900;
constexpr int SIDE = 100;
constexpr int COLUMNS = SIDE;
constexpr int ROWS = SIDE;
constexpr int WIDTH = 600;
constexpr int HEIGHT = 600;
constexpr float g = 2000;
constexpr int RADIUS = 10;
constexpr float DAMPING = 0.98;
constexpr int PASSES = 60;
constexpr int FIBERLENGTH = 5;


constexpr int NUMBEROFPOINTS = ROWS * COLUMNS;
constexpr int RADIUSSQUARED = RADIUS * RADIUS;
constexpr int SPACINGX = WIDTH / (float)(COLUMNS - 1);
constexpr int SPACINGY = HEIGHT / (float)(ROWS - 1);
constexpr int LENGTHX = SPACINGX;
constexpr int LENGTHY = SPACINGY;
constexpr float STARTX = (SCREENX - WIDTH) / 2.0f;
constexpr float STARTY = (SCREENY - HEIGHT) / 2.0f;

std::array<float, NUMBEROFPOINTS> CURRENTPOSITIONSX;
std::array<float, NUMBEROFPOINTS> CURRENTPOSITIONSY;
std::array<float, NUMBEROFPOINTS> PREVIOUSPOSITIONSX;
std::array<float, NUMBEROFPOINTS> PREVIOUSPOSITIONSY;
std::array<int, ROWS* COLUMNS> VERTICALSTICKSCONNECTED;
std::array<int, ROWS* COLUMNS> HORIZONTALSTICKSCONNECTED;


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


void ApplyGravityToPoints() {
	float GRAVITY = g * DELTATIME * DELTATIME;
	for (int POINT = 0; POINT < NUMBEROFPOINTS; POINT++) {
		if (POINT < COLUMNS) {
			continue;
		}
		float VELOCITYX = CURRENTPOSITIONSX[POINT] - PREVIOUSPOSITIONSX[POINT];
		float VELOCITYY = CURRENTPOSITIONSY[POINT] - PREVIOUSPOSITIONSY[POINT];
		float TEMPORARYX = CURRENTPOSITIONSX[POINT];
		float TEMPORARYY = CURRENTPOSITIONSY[POINT];
		CURRENTPOSITIONSX[POINT] += VELOCITYX * DAMPING;
		CURRENTPOSITIONSY[POINT] += VELOCITYY * DAMPING + GRAVITY;
		PREVIOUSPOSITIONSX[POINT] = TEMPORARYX;
		PREVIOUSPOSITIONSY[POINT] = TEMPORARYY;
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
	WORKSEMAPHORE.release(COLUMNS / FIBERLENGTH);
	DONESEMAPHORE.acquire();
}

void EvaluateFibers() {
	while (RUNNING) {
		WORKSEMAPHORE.acquire();
		int INDEX = NEXTWORK.fetch_add(FIBERLENGTH);
		if (WORKALONG == 'X') {
			int CURRENTPOINT = INDEX * COLUMNS;
			int LASTPOINT = CURRENTPOINT + FIBERLENGTH * COLUMNS - 1;
			for (CURRENTPOINT; CURRENTPOINT <= LASTPOINT; CURRENTPOINT++) {
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
			int ROW = -1;
			int CURRENTPOINT = INDEX;
			int LASTPOINT = CURRENTPOINT + (COLUMNS * (ROWS - 1)) + (FIBERLENGTH - 1);
			for (CURRENTPOINT; CURRENTPOINT < LASTPOINT; (CURRENTPOINT = NextVerticalPoint(CURRENTPOINT))) {
				ROW++;
				int NEXTPOINT = NextVerticalPoint(CURRENTPOINT);
				if (ROW == ROWS) {
					ROW = 0;
				}
				if (!VERTICALSTICKSCONNECTED[CURRENTPOINT]) {
					continue;
				}
				float dx = CURRENTPOSITIONSX[NEXTPOINT] - CURRENTPOSITIONSX[CURRENTPOINT];
				float dy = CURRENTPOSITIONSY[NEXTPOINT] - CURRENTPOSITIONSY[CURRENTPOINT];
				float DISTANCE = std::sqrt(dx * dx + dy * dy);
				float CORRECTIONRATIO = (DISTANCE - LENGTHY) / DISTANCE;
				if (ROW > 0) {
					float CORRECTION = CORRECTIONRATIO / 2;
					CURRENTPOSITIONSX[CURRENTPOINT] = CURRENTPOSITIONSX[CURRENTPOINT] + dx * CORRECTION;
					CURRENTPOSITIONSY[CURRENTPOINT] = CURRENTPOSITIONSY[CURRENTPOINT] + dy * CORRECTION;
					CURRENTPOSITIONSX[NEXTPOINT] = CURRENTPOSITIONSX[NEXTPOINT] - dx * CORRECTION;
					CURRENTPOSITIONSY[NEXTPOINT] = CURRENTPOSITIONSY[NEXTPOINT] - dy * CORRECTION;
				}
				else if (ROW == 0) {
					float CORRECTION = CORRECTIONRATIO;
					CURRENTPOSITIONSX[NEXTPOINT] = CURRENTPOSITIONSX[NEXTPOINT] - dx * CORRECTION;
					CURRENTPOSITIONSY[NEXTPOINT] = CURRENTPOSITIONSY[NEXTPOINT] - dy * CORRECTION;
				}
			}
			if (WORKDONE.fetch_add(FIBERLENGTH) == COLUMNS - FIBERLENGTH) {
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