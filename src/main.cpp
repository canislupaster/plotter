#include <Arduino.h>
#include <Servo.h>

const int shield_pin = 8, servo_pin=11;

struct Pen {
	Servo pen;
	bool is_up=false;

	unsigned long pen_last_down;
	bool can_move;
	const unsigned long pen_delay = 100;

	void init() {
		pen.attach(servo_pin), up();
	}

	void up() {
		if (is_up) return;

		pen.write(0);
		pen_last_down=millis();
		can_move=false;
		is_up=true;
		Serial.println("pen up");
	}

	void down() {
		if (!is_up) return;

		pen.write(90);
		pen_last_down=millis();
		can_move=false;
		is_up=false;
		Serial.println("pen down");
	}

	bool check_can_move() {
		if (!can_move)
			can_move=millis()-pen_last_down >= pen_delay;
		return can_move;
	}
};

template<int pin, int dir_pin>
struct Stepper {
	static const int pulse_delay = 1;

	void init() {
		pinMode(pin, OUTPUT);
		pinMode(dir_pin, OUTPUT);
	}

	void do_step(bool dir) {
		digitalWrite(dir_pin, dir);
		delayMicroseconds(pulse_delay);
		digitalWrite(pin, HIGH);
		delayMicroseconds(pulse_delay);
		digitalWrite(pin, LOW);
	}
};

struct Pt: Printable {
	long x, y;

	Pt(): x(0), y(0) {}
	Pt(long x_, long y_): x(x_), y(y_) {}

	bool operator==(Pt const& o) const { return x==o.x && y==o.y; }
	bool operator!=(Pt const& o) const { return x!=o.x || y!=o.y; }
	Pt operator+(Pt const& o) const { return {x+o.x, y+o.y}; }
	Pt operator-(Pt const& o) const { return {x-o.x, y-o.y}; }

	size_t printTo(Print& p) const {
		size_t n = 0;
		n += p.print("(");
		n += p.print(x);
		n += p.print(",");
		n += p.print(y);
		n += p.print(")");
		return n;
	}
};

const Pt limit(29875, 24421);
const int default_delay = 10000;

struct Save {
	Pt cur, to;
	unsigned long us;
};

struct Cur {
	Pt from, true_from;
	Pt to;

	Pt cur, true_cur, diff;
	unsigned long us=default_delay;
	unsigned long last_step=0;

	Pen* p = nullptr;

	bool moving=true, should_lock=false;

	void init_from(Save save) {
		from=cur, true_from=true_cur;
		to=save.to, us=save.us;
		diff = Pt(to.x - to.y, to.x + to.y)-true_cur;
		set_moving(true);
	}

	Save save() const { return {cur, to, us}; }

	void set_moving(bool moving_) {
		if (moving!=moving_) {
			digitalWrite(shield_pin, moving_ || should_lock ? LOW : HIGH);
			moving=moving_;
		}
	}

	void lock() {
		if (!should_lock && !moving)
			digitalWrite(shield_pin, LOW);
		should_lock=true;
	}

	void unlock() {
		if (!moving && should_lock)
			digitalWrite(shield_pin, HIGH);
		should_lock=false;
	}

	void reset() {
		cur = {0,0}, true_cur = {0,0}, to={0,0};
		set_moving(false);
		us=default_delay;
	}

	void init(Pt to_, int speed) {
		from=cur, true_from = true_cur;
		to=Pt(constrain(to_.x, 0, limit.x), constrain(to_.y, 0, limit.y));

		if (speed==0) {
			set_moving(false);
			us = default_delay;
			return;
		}
		
		if (cur==to) {
			us=1000000ll/speed;
			return;
		}
		
		set_moving(true);

		diff = Pt(to.x - to.y, to.x + to.y)-true_cur;

		//delay between steps
		long step_len_sq = (long)(to.x-cur.x)*(to.x-cur.x) + (long)(to.y-cur.y)*(to.y-cur.y);
		long long manhattan_len = abs(diff.x) + abs(diff.y);

		long l=1, r=manhattan_len;
		while (l<r) {
			long m = (l+r)/2;
			if (m*m < step_len_sq) l=m+1;
			else r=m;
		}

		us = (1000000ll*l)/(manhattan_len*speed);

		// Serial.print("from "), Serial.print(from), Serial.print(", to "), Serial.print(to), Serial.print(", us "), Serial.println(us);
	}

	void delay() {
		unsigned long now = micros();
		if (us>now-last_step) delayMicroseconds(us-(now-last_step));
		last_step = now;
	}
	
	Pt step_dir() {
		if (cur==to || !p->check_can_move()) return Pt(0,0);

		Pt ret(0,0);

		if (diff.x==0) ret.y=diff.y>0 ? 1 : -1;
		else if (diff.y==0) ret.x=diff.x>0 ? 1 : -1;
		else {
			Pt cd = true_cur - true_from;
			//this just barely fits lmao but family friendly
			bool det = (long)diff.x*cd.y>=(long)diff.y*cd.x;
			int quadrant = diff.x>0 ? (diff.y>0 ? 0 : 3) : (diff.y>0 ? 1 : 2);

			if (det) {
				if (quadrant%2 == 0) ret.x=1-quadrant;
				else ret.y = 2-quadrant;
			} else {
				if (quadrant%2 == 1) ret.x=quadrant-2;
				else ret.y = 1-quadrant;
			}
		}
		
		true_cur = true_cur + ret;
		Pt floor_cur = Pt((true_cur.x + true_cur.y)/2, (true_cur.y - true_cur.x)/2);
		if ((true_cur.x+true_cur.y)%2 == 0) cur=floor_cur;

		//can exceed limits by 1/2 step
		if (floor_cur.x<0 || floor_cur.y<0 || floor_cur.x>limit.x || floor_cur.y>limit.y) {
			Serial.println("out of bounds!");
			init(cur, 0);
			return {0,0};
		}

		return ret;
	}
};

const int npt=300;
const int default_speed=600;
const int move_speed=700;

Save paused;

struct Move {
	enum Ty {Linear, Bezier, None} ty;
	Pt from, ctrl1, ctrl2, to;
	int speed; //steps/s
	
	int t=0;
	bool started=false, stay_down;
	bool is_paused=false, restore_cur=false;

	Move(Ty ty, Pt from, Pt ctrl1, Pt ctrl2, Pt to, int speed, int t, bool stay_down):
		ty(ty), from(from), ctrl1(ctrl1), ctrl2(ctrl2), to(to), speed(speed), t(t), stay_down(stay_down) {}

	static Move linear(Pt* pts, int speed, bool stay_down) {
		return Move(Linear, pts[0], {0,0}, {0,0}, pts[1], speed, 0, stay_down);
	}

	static Move bezier(Pt* pts, int speed, bool stay_down) {
		return Move(Bezier, pts[0], pts[1], pts[2], pts[3], speed, 0, stay_down);
	}

	void bezier_next() {
		Pt nxt;
		do {
			if (t>=npt) {
				cur.init(to, speed);
				return;
			}

			t++;

			long long x = (long long)from.x*(npt-t)*(npt-t)*(npt-t) + 3*(long long)ctrl1.x*(npt-t)*(npt-t)*t + 3*(long long)ctrl2.x*(npt-t)*t*t + (long long)to.x*t*t*t;
			x/=(long long)npt*npt*npt;
			long long y = (long long)from.y*(npt-t)*(npt-t)*(npt-t) + 3*(long long)ctrl1.y*(npt-t)*(npt-t)*t + 3*(long long)ctrl2.y*(npt-t)*t*t + (long long)to.y*t*t*t;
			y/=(long long)npt*npt*npt;

			nxt = {static_cast<long>(x),static_cast<long>(y)};
		} while (nxt==cur.cur);

		cur.init(nxt, speed);
	}

	void next() {
		if (is_paused) return;

		if (restore_cur) {
			cur.init_from(paused);
			restore_cur=false;
		}

		if (!started) {
			started=true;

			if (cur.cur!=from) {
				p.up();
				cur.init(from, move_speed);
			} else {
				next();
			}

			return;
		}

		if (ty==Move::Bezier) bezier_next();
		else if (cur.cur!=to) cur.init(to, speed);

		if ((ty==Move::Linear || t>=npt) && cur.cur==to) {
			if (!stay_down) p.up();
			Serial.println("done");
			ty=None;
		} else {
			p.down();
		}
	}

	void pause() {
		is_paused=true;
		if (started && cur.cur!=cur.to) {
			paused=cur.save();
			restore_cur=true;

			cur.init(cur.cur, 0);
		}
	}

	void resume() {
		if (!is_paused) return;

		is_paused=false;
		if (restore_cur) {
			if (paused.cur!=cur.cur) cur.init(paused.cur, speed);
			else cur.init_from(paused), restore_cur=false;
		}

		if (!restore_cur) next();
	}
};

Move move;

Stepper<2,5> x;
Stepper<3,6> y;

Pen p;
Cur cur;

void setup() {
	Serial.println("init");
	Serial.setTimeout(0);
	
	cur.p = &p;
	cur.init({0,0}, 0);
	move.ty = Move::None;

	Serial.begin(9600);
	x.init(), y.init(), p.init();

	pinMode(shield_pin, OUTPUT);
}

struct Parser {
	char const* x;
	bool bad=false;
	Parser(char const* x_): x(x_) {}

	void skip_ws() {
		while (*x && isWhitespace(*x)) x++;
	}

	bool starts_with(char const* y) {
		if (strlen(x)>=strlen(y) && strncmp(x, y, strlen(y))==0) {
			x+=strlen(y);
			return true;
		} else return false;
	}

	Pt parse_pt() {
		Pt o;
		skip_ws();
		o.x = strtol(x, const_cast<char**>(&x), 10);

		if (*x != ',') {
			Serial.println("expected comma");
			bad=true;
			return o;
		}

		x++;
		o.y = strtol(x, const_cast<char**>(&x), 10);
	}

	void expect_end() {
		if (*x=='\r') x++;
		if (*x) {
			Serial.print("expected end of input, got \"");
			Serial.print(x), Serial.println("\"");
			bad=true;
			return;
		}
	}
};

template<typename T, size_t N>
struct Dequeue {
	using n = N;

	T arr[n];
	int head=0, tail=0;

	void push(T x) {
		arr[tail++]=x;
		if (tail==n) tail=0;
	}

	T pop() {
		T ret = arr[head++];
		if (head==n) head=0;
		return ret;
	}

	void clear() { head=tail; }

	bool empty() const { return head==tail; }
	bool full() const { return tail+1==head || (head==0 && tail==n-1); }
};

unsigned long input_last = 0;
const unsigned long input_interval = 500;
char inp_buf[200];
char* inp_pt=inp_buf;

Dequeue<Move, 5> next_moves;

void handle_input() {
	if (millis()-input_last > input_interval) {
		input_last = millis();

		Serial.print("STATE ");
		Serial.print(cur.cur), Serial.print(" "), Serial.print(cur.from),
			Serial.print(" "), Serial.print(cur.to),
			Serial.print(" "), Serial.println(!p.is_up);
	}

	if (Serial.available()==0) return;

	bool end=false;
	while (inp_pt < inp_buf + sizeof(inp_buf)) {
		int c = Serial.read();

		if (c<0) break;
		else if (c=='\n') {end=true; break;}

		*(inp_pt++) = c;
	}

	if (!end) {
		if (inp_pt == inp_buf + sizeof(inp_buf)) {
			Serial.println("input buffer full, retry");
			inp_pt = inp_buf;
		}

		return;
	}

	*inp_pt = 0, inp_pt = inp_buf;
	// Serial.print("got \""), Serial.print(inp_buf); Serial.println("\"");

	Parser parse(inp_buf);
	parse.skip_ws();

	bool cancel = parse.starts_with("cancel");
	bool go = parse.starts_with("go");

	if (cancel || parse.starts_with("reset")) {
		move.ty = Move::None;
		next_moves.clear();
		p.up();

		if (!cancel) cur.reset();
		else cur.init(cur.cur, 0);
	} else if (parse.starts_with("lock")) {
		cur.lock();
	} else if (parse.starts_with("unlock")) {
		cur.unlock();

	} else if (parse.starts_with("pause")) {
		if (move.ty!=Move::None) move.pause();
	} else if (parse.starts_with("resume")) {
		if (move.ty!=Move::None) move.resume();

	//jogging only when no active move
	} else if (go || parse.starts_with("halt")) {
		if (move.ty!=Move::None) {
			Serial.println("busy");
			return;
		}

		if (go) {
			Pt to = parse.parse_pt();
			if (parse.bad) return;

			cur.init(to, move_speed);
		} else {
			cur.init(cur.cur, 0);
		}

	} else {
		bool is_line = parse.starts_with("l");

		Pt pts[4];
		for (int i=0; i<is_line ? 2 : 4; i++) {
			parse.skip_ws();
			pts[i] = parse.parse_pt();
		}

		bool stay_down = parse.starts_with("stay_down");
		parse.skip_ws(), parse.expect_end();

		if (parse.bad) return;
		
		if (next_moves.full()) {
			Serial.println("busy");
			return;
		}

		if (is_line) next_moves.push(Move::linear(pts, default_speed, stay_down));
		else next_moves.push(Move::bezier(pts, default_speed, stay_down));

		// Move& next = next_moves.arr[next_moves.tail==0 ? next_moves.n-1 : next_moves.tail-1];
		// Serial.println("got move");
		// Serial.print("from: "), Serial.print(next.from), Serial.print(", ctrl1: "), Serial.print(next.ctrl1), Serial.print(", ctrl2: "), Serial.print(next.ctrl2), Serial.print(", to: "), Serial.print(next.to), Serial.print(", stay_down: "), Serial.println(next.stay_down);
	}
}

void loop() {
	handle_input();

	do {
		if (move.ty==Move::None && !next_moves.empty()) {
			move = next_moves.pop();
		}

		if (cur.cur==cur.to) {
			if (move.ty==Move::None) {
				cur.init(cur.cur, 0);
				break;
			} else {
				move.next();
			}
		} else {
			break;
		}
	} while (move.ty==Move::None); //after move.next(), we loop for another move

	cur.delay();

	Pt dir = cur.step_dir();
	if (dir.x!=0) x.do_step(dir.x>0); 
	else if (dir.y!=0) y.do_step(dir.y>0);
}