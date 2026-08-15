// 디지털핀 설계
#define A_F_pin 4    // 4번 디지털핀(left_A_pin)   :: 앞바퀴 왼쪽 앞으로
#define A_B_pin 7    // 7번 디지털핀(left_B_pin)   :: 앞바퀴 왼쪽 뒤로
#define B_F_pin 12  // 12번 디지털핀(right_A_pin)  :: 앞바퀴 오른쪽 앞으로
#define B_B_pin 8   // 8번 디지털핀(right_B_pin)   :: 앞바퀴 오른쪽 뒤로
#define C_F_pin 2   // 2번 디지털핀(left_A_pin)  :: 뒷바퀴 왼쪽 앞으로
#define C_B_pin 3   // 3번 디지털핀(left_B_pin)  :: 뒷바퀴 왼쪽 뒤로
#define D_F_pin 23  // 23번 디지털핀(right_A_pin)  :: 뒷바퀴 오른쪽 앞으로
#define D_B_pin 22  // 22번 디지털핀(right_B_pin)  :: 뒷바퀴 오른쪽 뒤로

#define F_left_PWM_pin 9                // 9번 디지털핀(left_A_pin)              :: MOTER PWM
#define F_right_PWM_pin 10              // 10번 디지털핀(left_B_pin)              :: MOTER PWM
#define B_left_PWM_pin 5                // 5번 디지털핀(right_A_pin)             :: MOTER PWM
#define B_right_PWM_pin 11              // 11번 디지털핀(right_B_pin)             :: MOTER PWM


void motor_output()
{
  pinMode(A_F_pin, OUTPUT);  // left_A_pin
  pinMode(A_B_pin, OUTPUT);  // left_B_pin
  pinMode(B_F_pin, OUTPUT); // right_A_pin
  pinMode(B_B_pin, OUTPUT); // right_B_pin
  pinMode(C_F_pin, OUTPUT);  // left_A_pin
  pinMode(C_B_pin, OUTPUT);  // left_B_pin
  pinMode(D_F_pin, OUTPUT); // right_A_pin
  pinMode(D_B_pin, OUTPUT); // right_B_pin  
}


// 메카넘휠 조종 함수
// MOTER 구동 함수

// 전진 함수
void move_Forward(int Moter_PWM_value)  
{
	// 앞 모터
	digitalWrite(A_F_pin, HIGH);
	digitalWrite(A_B_pin, LOW);
  analogWrite(F_left_PWM_pin, Moter_PWM_value);
	digitalWrite(B_F_pin, HIGH);
	digitalWrite(B_B_pin, LOW);
  analogWrite(F_right_PWM_pin, Moter_PWM_value);
	// 뒤 모터
	digitalWrite(C_F_pin, HIGH);
	digitalWrite(C_B_pin, LOW);
  analogWrite(B_left_PWM_pin, Moter_PWM_value);
	digitalWrite(D_F_pin, HIGH);
	digitalWrite(D_B_pin, LOW);
  analogWrite(B_right_PWM_pin, Moter_PWM_value);
}

// 후진 함수
void move_Backward(int Moter_PWM_value)  
{
	// 앞 모터
	digitalWrite(A_F_pin, LOW);
	digitalWrite(A_B_pin, HIGH);
  analogWrite(F_left_PWM_pin, Moter_PWM_value);
	digitalWrite(B_F_pin, LOW);
	digitalWrite(B_B_pin, HIGH);
  analogWrite(F_right_PWM_pin, Moter_PWM_value);
	// 뒤 모터
	digitalWrite(C_F_pin, LOW);
	digitalWrite(C_B_pin, HIGH);
  analogWrite(B_left_PWM_pin, Moter_PWM_value);
	digitalWrite(D_F_pin, LOW);
	digitalWrite(D_B_pin, HIGH);
  analogWrite(B_right_PWM_pin, Moter_PWM_value);
}

// 왼쪽 이동함수
void move_Leftward(int Moter_PWM_value)
{
	// 앞 모터
	digitalWrite(A_F_pin, LOW);
	digitalWrite(A_B_pin, HIGH);
  analogWrite(F_left_PWM_pin, Moter_PWM_value);  
	digitalWrite(B_F_pin, HIGH);
	digitalWrite(B_B_pin, LOW);
  analogWrite(F_right_PWM_pin, Moter_PWM_value);
	// 뒤 모터
	digitalWrite(C_F_pin, HIGH);
	digitalWrite(C_B_pin, LOW);
  analogWrite(B_left_PWM_pin, Moter_PWM_value);
	digitalWrite(D_F_pin, LOW);
	digitalWrite(D_B_pin, HIGH);
  analogWrite(B_right_PWM_pin, Moter_PWM_value);
}

// 오른쪽 이동함수
void move_Rightward(int Moter_PWM_value)
{
	// 앞 모터
	digitalWrite(A_F_pin, HIGH);
	digitalWrite(A_B_pin, LOW);
  analogWrite(F_left_PWM_pin, Moter_PWM_value);
	digitalWrite(B_F_pin, LOW);
	digitalWrite(B_B_pin, HIGH);
  analogWrite(F_right_PWM_pin, Moter_PWM_value);
	// 뒤 모터
	digitalWrite(C_F_pin, LOW);
	digitalWrite(C_B_pin, HIGH);
  analogWrite(B_left_PWM_pin, Moter_PWM_value);
	digitalWrite(D_F_pin, HIGH);
	digitalWrite(D_B_pin, LOW);
  analogWrite(B_right_PWM_pin, Moter_PWM_value);
}

// 왼쪽대각선전진 이동함수
void move_LeftFordia(int Moter_PWM_value)
{
	// 앞 모터
	digitalWrite(A_F_pin, LOW);
	digitalWrite(A_B_pin, LOW);
  analogWrite(F_left_PWM_pin, Moter_PWM_value);
	digitalWrite(B_F_pin, HIGH);
	digitalWrite(B_B_pin, LOW);
  analogWrite(F_right_PWM_pin, Moter_PWM_value);
	// 뒤 모터
	digitalWrite(C_F_pin, HIGH);
	digitalWrite(C_B_pin, LOW);
  analogWrite(B_left_PWM_pin, Moter_PWM_value);
	digitalWrite(D_F_pin, LOW);
	digitalWrite(D_B_pin, LOW);
  analogWrite(B_right_PWM_pin, Moter_PWM_value);
}

// 오른쪽대각선전진 이동함수
void move_RightFordia(int Moter_PWM_value)
{
	// 앞 모터
	digitalWrite(A_F_pin, HIGH);
	digitalWrite(A_B_pin, LOW);
  analogWrite(F_left_PWM_pin, Moter_PWM_value);
	digitalWrite(B_F_pin, LOW);
	digitalWrite(B_B_pin, LOW);
  analogWrite(F_right_PWM_pin, Moter_PWM_value);
	// 뒤 모터
	digitalWrite(C_F_pin, LOW);
	digitalWrite(C_B_pin, LOW);
  analogWrite(B_left_PWM_pin, Moter_PWM_value);
	digitalWrite(D_F_pin, HIGH);
	digitalWrite(D_B_pin, LOW);
  analogWrite(B_right_PWM_pin, Moter_PWM_value);
}

// 왼쪽대각선후진 이동함수
void move_LeftBackdia(int Moter_PWM_value)
{
	// 앞 모터
	digitalWrite(A_F_pin, LOW);
	digitalWrite(A_B_pin, HIGH);
  analogWrite(F_left_PWM_pin, Moter_PWM_value);
	digitalWrite(B_F_pin, LOW);
	digitalWrite(B_B_pin, LOW);
  analogWrite(F_right_PWM_pin, Moter_PWM_value);
	// 뒤 모터
	digitalWrite(C_F_pin, LOW);
	digitalWrite(C_B_pin, LOW);
  analogWrite(B_left_PWM_pin, Moter_PWM_value);
	digitalWrite(D_F_pin, LOW);
	digitalWrite(D_B_pin, HIGH);
  analogWrite(B_right_PWM_pin, Moter_PWM_value);
}

// 오른쪽대각선후진 이동함수
void move_RightBackdia(int Moter_PWM_value)
{
	// 앞 모터
	digitalWrite(A_F_pin, LOW);
	digitalWrite(A_B_pin, LOW);
  analogWrite(F_left_PWM_pin, Moter_PWM_value);
	digitalWrite(B_F_pin, LOW);
	digitalWrite(B_B_pin, HIGH);
  analogWrite(F_right_PWM_pin, Moter_PWM_value);
	// 뒤 모터
	digitalWrite(C_F_pin, LOW);
	digitalWrite(C_B_pin, HIGH);
  analogWrite(B_left_PWM_pin, Moter_PWM_value);
	digitalWrite(D_F_pin, LOW);
	digitalWrite(D_B_pin, LOW);
  analogWrite(B_right_PWM_pin, Moter_PWM_value);
}

// 왼쪽코너링전진 이동함수
void move_LeftForCornering(int Moter_PWM_value)
{
	// 앞 모터
	digitalWrite(A_F_pin, LOW);
	digitalWrite(A_B_pin, LOW);
  analogWrite(F_left_PWM_pin, Moter_PWM_value);
	digitalWrite(B_F_pin, HIGH);
	digitalWrite(B_B_pin, LOW);
  analogWrite(F_right_PWM_pin, Moter_PWM_value);
	// 뒤 모터
	digitalWrite(C_F_pin, LOW);
	digitalWrite(C_B_pin, LOW);
  analogWrite(B_left_PWM_pin, Moter_PWM_value);
	digitalWrite(D_F_pin, HIGH);
	digitalWrite(D_B_pin, LOW);
  analogWrite(B_right_PWM_pin, Moter_PWM_value);
}

// 오른쪽코너링전진 이동함수
void move_RightForCornering(int Moter_PWM_value)
{
	// 앞 모터
	digitalWrite(A_F_pin, HIGH);
	digitalWrite(A_B_pin, LOW);
  analogWrite(F_left_PWM_pin, Moter_PWM_value);
	digitalWrite(B_F_pin, LOW);
	digitalWrite(B_B_pin, LOW);
  analogWrite(F_right_PWM_pin, Moter_PWM_value);
	// 뒤 모터
	digitalWrite(C_F_pin, HIGH);
	digitalWrite(C_B_pin, LOW);
  analogWrite(B_left_PWM_pin, Moter_PWM_value);
	digitalWrite(D_F_pin, LOW);
	digitalWrite(D_B_pin, LOW);
  analogWrite(B_right_PWM_pin, Moter_PWM_value);
}

// 왼쪽코너링후진 이동함수
void move_LeftBackCornering(int Moter_PWM_value)
{
	// 앞 모터
	digitalWrite(A_F_pin, LOW);
	digitalWrite(A_B_pin, LOW);
  analogWrite(F_left_PWM_pin, Moter_PWM_value);
	digitalWrite(B_F_pin, LOW);
	digitalWrite(B_B_pin, HIGH);
  analogWrite(F_right_PWM_pin, Moter_PWM_value);
	// 뒤 모터
	digitalWrite(C_F_pin, LOW);
	digitalWrite(C_B_pin, LOW);
  analogWrite(B_left_PWM_pin, Moter_PWM_value);
	digitalWrite(D_F_pin, LOW);
	digitalWrite(D_B_pin, HIGH);
  analogWrite(B_right_PWM_pin, Moter_PWM_value);
}

// 오른쪽코너링후진 이동함수
void move_RightBackCornering(int Moter_PWM_value)
{
	// 앞 모터
	digitalWrite(A_F_pin, LOW);
	digitalWrite(A_B_pin, HIGH);
  analogWrite(F_left_PWM_pin, Moter_PWM_value);
	digitalWrite(B_F_pin, LOW);
	digitalWrite(B_B_pin, LOW);
  analogWrite(F_right_PWM_pin, Moter_PWM_value);
	// 뒤 모터
	digitalWrite(C_F_pin, LOW);
	digitalWrite(C_B_pin, HIGH);
  analogWrite(B_left_PWM_pin, Moter_PWM_value);
	digitalWrite(D_F_pin, LOW);
	digitalWrite(D_B_pin, LOW);
  analogWrite(B_right_PWM_pin, Moter_PWM_value);
}

// 왼쪽라운딩 이동함수
void move_LeftTurnRound(int Moter_PWM_value)
{
	// 앞 모터
	digitalWrite(A_F_pin, LOW);
	digitalWrite(A_B_pin, HIGH);
  analogWrite(F_left_PWM_pin, Moter_PWM_value);
	digitalWrite(B_F_pin, HIGH);
	digitalWrite(B_B_pin, LOW);
  analogWrite(F_right_PWM_pin, Moter_PWM_value);
	// 뒤 모터
	digitalWrite(C_F_pin, LOW);
	digitalWrite(C_B_pin, HIGH);
  analogWrite(B_left_PWM_pin, Moter_PWM_value);
	digitalWrite(D_F_pin, HIGH);
	digitalWrite(D_B_pin, LOW);
  analogWrite(B_right_PWM_pin, Moter_PWM_value);
}

// 오른쪽라운딩 이동함수
void move_RightTurnRound(int Moter_PWM_value)
{
	// 앞 모터
	digitalWrite(A_F_pin, HIGH);
	digitalWrite(A_B_pin, LOW);
  analogWrite(F_left_PWM_pin, Moter_PWM_value);
	digitalWrite(B_F_pin, LOW);
	digitalWrite(B_B_pin, HIGH);
  analogWrite(F_right_PWM_pin, Moter_PWM_value);
	// 뒤 모터
	digitalWrite(C_F_pin, HIGH);
	digitalWrite(C_B_pin, LOW);
  analogWrite(B_left_PWM_pin, Moter_PWM_value);
	digitalWrite(D_F_pin, LOW);
	digitalWrite(D_B_pin, HIGH);
  analogWrite(B_right_PWM_pin, Moter_PWM_value);
}

// 왼쪽라운딩(뒤축기준) 이동함수
void move_LeftTurnRoundRearAxis(int Moter_PWM_value)
{
	// 앞 모터
	digitalWrite(A_F_pin, LOW);
	digitalWrite(A_B_pin, HIGH);
  analogWrite(F_left_PWM_pin, Moter_PWM_value);
	digitalWrite(B_F_pin, HIGH);
	digitalWrite(B_B_pin, LOW);
  analogWrite(F_right_PWM_pin, Moter_PWM_value);
	// 뒤 모터
	digitalWrite(C_F_pin, LOW);
	digitalWrite(C_B_pin, LOW);
  analogWrite(B_left_PWM_pin, Moter_PWM_value);
	digitalWrite(D_F_pin, LOW);
	digitalWrite(D_B_pin, LOW);
  analogWrite(B_right_PWM_pin, Moter_PWM_value);
}

// 오른쪽라운딩(뒤축기준) 이동함수
void move_RightTurnRoundRearAxis(int Moter_PWM_value)
{
	// 앞 모터
	digitalWrite(A_F_pin, HIGH);
	digitalWrite(A_B_pin, LOW);
  analogWrite(F_left_PWM_pin, Moter_PWM_value);
	digitalWrite(B_F_pin, LOW);
	digitalWrite(B_B_pin, HIGH);
  analogWrite(F_right_PWM_pin, Moter_PWM_value);
	// 뒤 모터
	digitalWrite(C_F_pin, LOW);
	digitalWrite(C_B_pin, LOW);
  analogWrite(B_left_PWM_pin, Moter_PWM_value);
	digitalWrite(D_F_pin, LOW);
	digitalWrite(D_B_pin, LOW);
  analogWrite(B_right_PWM_pin, Moter_PWM_value);
}

// 왼쪽라운딩(앞축기준) 이동함수
void move_LeftTurnRoundFrontAxis(int Moter_PWM_value)
{
	// 앞 모터
	digitalWrite(A_F_pin, LOW);
	digitalWrite(A_B_pin, LOW);
  analogWrite(F_left_PWM_pin, Moter_PWM_value);
	digitalWrite(B_F_pin, LOW);
	digitalWrite(B_B_pin, LOW);
  analogWrite(F_right_PWM_pin, Moter_PWM_value);
	// 뒤 모터
	digitalWrite(C_F_pin, HIGH);
	digitalWrite(C_B_pin, LOW);
  analogWrite(B_left_PWM_pin, Moter_PWM_value);
	digitalWrite(D_F_pin, LOW);
	digitalWrite(D_B_pin, HIGH);
  analogWrite(B_right_PWM_pin, Moter_PWM_value);
}

// 오른쪽라운딩(앞축기준) 이동함수
void move_RightTurnRoundFrondAxis(int Moter_PWM_value)
{
	// 앞 모터
	digitalWrite(A_F_pin, LOW);
	digitalWrite(A_B_pin, LOW);
  analogWrite(F_left_PWM_pin, Moter_PWM_value);
	digitalWrite(B_F_pin, LOW);
	digitalWrite(B_B_pin, LOW);
  analogWrite(F_right_PWM_pin, Moter_PWM_value);
	// 뒤 모터
	digitalWrite(C_F_pin, LOW);
	digitalWrite(C_B_pin, HIGH);
  analogWrite(B_left_PWM_pin, Moter_PWM_value);
	digitalWrite(D_F_pin, HIGH);
	digitalWrite(D_B_pin, LOW);
  analogWrite(B_right_PWM_pin, Moter_PWM_value);
}

// 정지 함수
void Stop_Moter()
{
	// 앞 모터
	digitalWrite(A_F_pin, LOW);
	digitalWrite(A_B_pin, LOW);
  //analogWrite(F_left_PWM_pin, Moter_PWM_value);
	digitalWrite(B_F_pin, LOW);
	digitalWrite(B_B_pin, LOW);
  //analogWrite(F_right_PWM_pin, Moter_PWM_value);
	// 뒤 모터
	digitalWrite(C_F_pin, LOW);
	digitalWrite(C_B_pin, LOW);
  //analogWrite(B_left_PWM_pin, Moter_PWM_value);
	digitalWrite(D_F_pin, LOW);
	digitalWrite(D_B_pin, LOW);
  //analogWrite(B_right_PWM_pin, Moter_PWM_value);
}

