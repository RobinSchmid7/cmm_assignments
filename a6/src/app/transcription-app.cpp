#ifdef WIN32
#define NOMINMAX
#endif
#include <application.h>
#include <imgui.h>
#include <imgui_multiplot.h>
#include <chrono>

#include <BernHelpers.h>
#include <GradientDescentMinimizer.h>
#include <NewtonFunctionMinimizer.h>
#include <ObjectiveFunction.h>

bool I_AM_HAVING_SCREEN_ISSUES = false; 
const int K = 20; // Number of timesteps (feel free to make smaller if code is running too slowly)

//////////////////////////////////////////////////////////////////////////////// 
typedef vector<Vector2d> Traj; 
VectorXd stackTraj_(const Traj &u) { VectorXd u_stack; u_stack.setZero(2 * u.size()); for (int i = 0; i < u.size(); ++i) { u_stack.segment(2 * i, 2) = u[i]; } return u_stack; }
Traj unstackTraj_(const VectorXd &u_stack) { Traj u; for (int i = 0; i < u_stack.size() / 2; ++i) { u.push_back(u_stack.segment(2 * i, 2)); } return u; } 
// (u0, ..., uK; x0, ..., xK, v0, ..., vK)
VectorXd assemble(const Traj &u, const Traj &x, const Traj &v) { return stack_({ stackTraj_(u), stackTraj_(x), stackTraj_(v) }); }
array<Traj, 3> disassemble(const VectorXd &stack) { int L = stack.size() / 3; return array<Traj, 3>({ unstackTraj_(stack.head(L)), unstackTraj_(stack.segment(L, L)), unstackTraj_(stack.tail(L)) }); } 
////////////////////////////////////////////////////////////////////////////////

Vector2d x0 = Vector2d(0., 0.);
Vector2d v0 = Vector2d(1., 0.);
Vector2d x_prime = Vector2d(0., 1.);
const double h = .033;
const double m = 1.;
bool SECOND_ORDER_SOLVER = true;

Vector2d get_Fk(const Vector2d &uk, const Vector2d &) { return uk; } 
pair<Vector2d, Vector2d> stepPhysicsExplicitEuler(const Vector2d &xk, const Vector2d &vk, const Vector2d &uk) {
	// NOTE: I use xkp1 to mean $x_{k+1}$.
	Vector2d xkp1 = xk;
	Vector2d vkp1 = vk;
	// TODO: xkp1 = ...
	// TODO: vkp1 = ...

    Vector2d Fk = get_Fk(uk, xk);
    xkp1 += h * vk;
    vkp1 += h * Fk/m;

//	cout << "NotImplementedError:stepPhysicsExplicitEuler" << endl;
    
	return std::make_pair(xkp1, vkp1);
}

bool DRAW_REAL_PHYSICS_FOR_COMPARISON = true;
pair<vector<Vector2d>, vector<Vector2d>> get_xv(const vector<Vector2d> &u) {
	// (x0, v0) --(u0, ..., uKm1)--> (x0, x1, ..., xK)
	vector<Vector2d> x = { x0 }; // x0, x1, ..., xK
	vector<Vector2d> v = { v0 };
	for (int k = 0; k < K; ++k) {
		auto xv_kp1 = stepPhysicsExplicitEuler(x[k], v[k], u[k]);
		x.push_back(xv_kp1.first);
		v.push_back(xv_kp1.second);
	}
	return make_pair(x, v);
}
vector<Vector2d> get_x(const vector<Vector2d> &u) { return get_xv(u).first; }
vector<Vector2d> get_v(const vector<Vector2d> &u) { return get_xv(u).second; }

////////////////////////////////////////////////////////////////////////////////

class TranscriptionObjective : public ObjectiveFunction {
public:

	// Please feel free to use this in the coefficient of a regularizer as pow(10., log_c_reg).
	// It's already in your GUI.

    // Weight coefficients
    double log_x0_reg = 3; double log_x0_reg_min = -5.; double log_x0_reg_max = 5.; // Initial position
    double log_v0_reg = 2.; double log_v0_reg_min = -5.; double log_v0_reg_max = 5.; // Initial velocity
    double log_xDyn_reg = 4.; double log_xDyn_reg_min = -5.; double log_xDyn_reg_max = 5.; // Position dynamics
    double log_vDyn_reg = 3.; double log_vDyn_reg_min = -5.; double log_vDyn_reg_max = 5.; // Velocity dynamics
    double log_xK_reg = 3.; double log_xK_reg_min = -5.; double log_xK_reg_max = 5.; // Final position
    double log_vK_reg = 2.; double log_vK_reg_min = -5.; double log_vK_reg_max = 5.; // Final velocity
    double log_xv_reg = -3.; double log_xv_reg_min = -5.; double log_xv_reg_max = 5.; // Stacked position and velocity

    virtual double evaluate(const VectorXd &stack) const {
		auto uxv = disassemble(stack);
		Traj u = uxv[0]; // u0, ..., uK
		Traj x = uxv[1]; // x0, ..., xK
		Traj v = uxv[2]; // v0, ..., vK
		// --
		double O = 0.; 
		// TODO: O += ...

		// Weights
        double x0_reg = pow(10., log_x0_reg);
        double v0_reg = pow(10., log_v0_reg);
        double xDyn_reg = pow(10., log_xDyn_reg);
        double vDyn_reg = pow(10., log_vDyn_reg);
        double xK_reg = pow(10., log_xK_reg);
        double vK_reg = pow(10., log_vK_reg);
        double xv_reg = pow(10., log_xv_reg);

        // Initial position
        O += x0_reg * (x0 - x[0]).squaredNorm();
        // Initial velocity
        O += v0_reg * (v0 - v[0]).squaredNorm();
        // Dynamics: x_{k+1} = x_{k} + h * v_{k}, v_{k+1} = v_{k} + h * u_{k}/m
        for (int k = 0; k < K; k++) {
            auto xvkp1 = stepPhysicsExplicitEuler(x[k], v[k], u[k]);
            O += xDyn_reg * (xvkp1.first - x[k+1]).squaredNorm();
            O += vDyn_reg * (xvkp1.second - v[k+1]).squaredNorm();
        }
		// Final position
		O += xK_reg * (x.back() - x_prime).squaredNorm();
		// Final velocity
		O += vK_reg * v.back().squaredNorm();
		// Regularizer
		O += xv_reg * stack.squaredNorm();
		
		return O;
	}
};

class TranscriptionApp : public Application {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	TranscriptionObjective objective = TranscriptionObjective();
	vector<Vector2d> u_curr; // u0, ..., uK
	vector<Vector2d> x_curr; // x0, ..., xK 
	vector<Vector2d> v_curr; // v0, ..., vK 
	vector<double> O_history;

public:
	bool OPTIMIZE = false;
	bool VISUALIZE = false;
	int k_visualize = -1;
	Vector2d u_visualize = Vector2d::Zero();
	Vector2d x_visualize = Vector2d::Zero();
	Vector2d v_visualize = Vector2d::Zero();
	void resetOptimization() {
		u_curr.clear(); { for (int _ = 0; _ <= K; ++_) { u_curr.push_back(Vector2d(-1, 1)); } }
		x_curr.clear(); { for (int _ = 0; _ <= K; ++_) { x_curr.push_back(x0 + _*h*v0); } }
		v_curr.clear(); { for (int _ = 0; _ <= K; ++_) { v_curr.push_back(v0); } }
	}

	void resetPlot(){
		O_history.clear(); 
		auto O0 = objective.evaluate(assemble(u_curr, x_curr, v_curr));
		for (int _ = 0; _ < 128; ++_) { O_history.push_back(O0); }
	}

	void resetVisualization() {
		k_visualize = -1;
	}
	bool SLOMO = false;
	bool PRINT_O = false;
	bool PRINT_magG = false;

public:
	TranscriptionApp(int widgth, int height, const char * title, float pixelRatio = 2.f) : Application(title, widgth, height) {
		ImGui::StyleColorsDark();
		// --
		resetOptimization();
		// --
		resetPlot();
		// --
		resetVisualization();
	}

public:
	void process() override {
		static std::chrono::high_resolution_clock::time_point lastFrame = std::chrono::high_resolution_clock::now();
		std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
		if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastFrame).count() >((SLOMO) ? 320 : 16)) {

			if (OPTIMIZE) {
				auto stack = assemble(u_curr, x_curr, v_curr);
				{
					if (!SECOND_ORDER_SOLVER) {
						GradientDescentLineSearch minimizer = GradientDescentLineSearch(1, 1e-10);
						minimizer.minimize(&objective, stack);
					} else {
						NewtonFunctionMinimizer minimizer = NewtonFunctionMinimizer(1, 1e-10);
						minimizer.minimize(&objective, stack);
					}
					push_back_pop_front(O_history, objective.evaluate(stack));
				}
				auto uxv_curr = disassemble(stack);
				u_curr = uxv_curr[0];
				x_curr = uxv_curr[1];
				v_curr = uxv_curr[2];
				if (PRINT_O)    { cout << " O  = " << objective.evaluate(stack) << endl; }
				if (PRINT_magG) { cout << "|G| = " << objective.getGradient(stack).norm() << endl; }
			}

			if (VISUALIZE) {
				k_visualize++;
				u_visualize = loop_access(u_curr, k_visualize);
				x_visualize = loop_access(x_curr, k_visualize);
				v_visualize = loop_access(v_curr, k_visualize);
			} else {
				resetVisualization();
			}

			lastFrame = now;
		}
	}

private:
	bool DRAW_THRUSTER = true;
	bool DRAW_VELOCITY = true;
	void drawNanoVG() override {
		strokeRect(Vector2d(0., 0.), Vector2d(1., 1.), GRAY);
		if (!VISUALIZE) {
			if (DRAW_REAL_PHYSICS_FOR_COMPARISON) {
				auto x_sim = get_x(u_curr);
				auto v_sim = get_v(u_curr);
				for (int k = 0; k < K; ++k) {
					if (DRAW_THRUSTER) { drawThruster(x_sim[k], u_curr[k], DARK_RED); }
					if (DRAW_VELOCITY) { drawVelocity(x_sim[k], v_sim[k], DARK_BLUE); }
					drawShip(x_sim[k], 0., LIGHT_GRAY);
				}
			}
			for (int k = 0; k <= K; ++k) {
				if (DRAW_THRUSTER) { drawThruster(x_curr[k], u_curr[k], RED); }
				if (DRAW_VELOCITY) { drawVelocity(x_curr[k], v_curr[k], BLUE); }
				drawShip(x_curr[k], 0., WHITE);
			}
			fillCircle(x_prime, PURPLE);
		} else {
			fillCircle(x_prime, PURPLE);
			if (DRAW_THRUSTER) { drawThruster(x_visualize, u_visualize, RED); }
			if (DRAW_VELOCITY) { drawVelocity(x_visualize, v_visualize, BLUE); }
			drawShip(x_visualize, 0., WHITE);
		}
	}

	ImColor PLOT_COLOR = ImColor(249, 38, 114);
	void drawImGui() override {
		using namespace ImGui;
		Begin("NOTE: Press r to reset (optimization or simulation).");
		Checkbox("OPTIMIZE // Keyboard Shortcut: o", &OPTIMIZE);
		SliderScalar("log_x0_reg", ImGuiDataType_Double, &objective.log_x0_reg, &objective.log_x0_reg_min, &objective.log_x0_reg_max);
        SliderScalar("log_v0_reg", ImGuiDataType_Double, &objective.log_v0_reg, &objective.log_v0_reg_min, &objective.log_v0_reg_max);
        SliderScalar("log_xDyn_reg", ImGuiDataType_Double, &objective.log_xDyn_reg, &objective.log_xDyn_reg_min, &objective.log_xDyn_reg_max);
        SliderScalar("log_vDyn_reg", ImGuiDataType_Double, &objective.log_vDyn_reg, &objective.log_vDyn_reg_min, &objective.log_vDyn_reg_max);
        SliderScalar("log_xK_reg", ImGuiDataType_Double, &objective.log_xK_reg, &objective.log_xK_reg_min, &objective.log_xK_reg_max);
        SliderScalar("log_vK_reg", ImGuiDataType_Double, &objective.log_vK_reg, &objective.log_vK_reg_min, &objective.log_vK_reg_max);
        SliderScalar("log_xv_reg", ImGuiDataType_Double, &objective.log_xv_reg, &objective.log_xv_reg_min, &objective.log_xv_reg_max);
        Checkbox("VISUALIZE // Keyboard Shortcut: s", &VISUALIZE);
		Checkbox("SLOMO",     &SLOMO);
		Checkbox("PRINT  O",  &PRINT_O);
		Checkbox("PRINT |G|", &PRINT_magG);
		Checkbox("DRAW_REAL_PHYSICS_FOR_COMPARISON", &DRAW_REAL_PHYSICS_FOR_COMPARISON);
		Checkbox("DRAW_THRUSTER", &DRAW_THRUSTER);
		Checkbox("DRAW_VELOCITY", &DRAW_VELOCITY);
		SliderScalar("ZOOM", ImGuiDataType_Double, &ZOOM_, &ZOOM_MIN_, &ZOOM_MAX_);
		{ // D:
			const char ** names = new const char*[1]; names[0] = "...";
			ImColor *colors = { &PLOT_COLOR };
			float **functionValues = new float*[1]; float *singleton = new float[O_history.size()]; for (int i = 0; i < O_history.size(); ++i) { singleton[i] = O_history[i]; } functionValues[0] = singleton;
			PlotMultiLines("O(u_curr)", 1, names, colors, [](const void *data, int idx)->float { return ((const float*)data)[idx]; }, ((const void *const *)functionValues), O_history.size(), O_history.size(), 0, 0., max_element(O_history), ImVec2(0, 150));
		}
		End();
	}

protected:
	bool DRAGGING = false;
	Vector2d x_mouse = { 0., 0. };
	void mouseButtonPressed(int, int) override {
		DRAGGING = ((x_mouse - x_prime).norm() < .1);
	}
	void mouseButtonReleased(int, int) override { DRAGGING = false; }
	void mouseMove(double, double) override {
		double f = (I_AM_HAVING_SCREEN_ISSUES) ? .5 : 1.; x_mouse = _2xy(Vector2d(f*mouseState.lastMouseX, f*mouseState.lastMouseY));
		// --
		if (DRAGGING) { x_prime = x_mouse; }
	}

	void keyPressed(int key, int mods) override {
		if (key == GLFW_KEY_O) { toggle(OPTIMIZE); VISUALIZE = false; }
		if (key == GLFW_KEY_S) { toggle(VISUALIZE); OPTIMIZE = false; }
		if (key == GLFW_KEY_R) {  resetOptimization(); resetPlot(); resetVisualization();    } // *
	}

private:
	double eps = .25; // * otherwise stuff doesn't show up
	double get_L_() { return double(std::min(height, width)); }
	double get_f_() { return get_L_() / (2. + 2 * eps); }
	double ZOOM_ = .4f;
	double ZOOM_MIN_ = .1f;
	double ZOOM_MAX_ = 2.f;
	double ZOOM() { return 1. / ZOOM_; }
	Vector2f _2nvg(Vector2d xy) {
		// zoom*(-1. - eps, 1. + eps) -x-> (0, L)
		// ""                         -y-> (L, 0)
		xy /= ZOOM();
		#if defined __APPLE__ 
		Vector2f ret = (get_f_() * (xy + (1. + eps)*Vector2d(1, 1.5))).cast<float>();
		#else
		Vector2f ret = (get_f_() * (xy + (1. + eps)*Vector2d::Ones())).cast<float>();
		#endif
		ret.y() = get_L_() - ret.y();
		return ret;
	}
	Vector2d _2xy(const Vector2f uv_) { return _2xy(Vector2d(uv_.cast<double>())); }
	Vector2d _2xy(Vector2d uv) {
		// zoom*(-1. - eps, 1. + eps) <-x- (0, L)
		//                    "" <-y- (L, ))
		uv.y() = get_L_() - uv.y();
		double _1of = 1. / get_f_();
		return ZOOM()*((_1of * uv) - (1. + eps) * Vector2d::Ones());
	}

	void drawShip(const Vector2d &s_, const double &theta, const NVGcolor &COLOR) { Vector2d o = .02*Vector2d::Ones(); fillRect(s_ - o, s_ + o, COLOR); }
	void drawThruster(const Vector2d &s_, const Vector2d &F_, const NVGcolor &COLOR) { drawVector(s_, -.1*F_, COLOR); }
	void drawVelocity(const Vector2d &s_, const Vector2d &v_, const NVGcolor &COLOR) { drawVector(s_, .1*v_, COLOR); }

	void strokeRect(const Vector2d &lr_, const Vector2d &LR_, const NVGcolor &COLOR) {
		Vector2f lr = _2nvg(lr_);
		Vector2f LR = _2nvg(LR_);
		Vector2f wh = LR - lr;
		// --
		nvgReset(vg);
		nvgBeginPath(vg);
		nvgRect(vg, lr.x(), lr.y(), wh.x(), wh.y());
		nvgStrokeColor(vg, COLOR);
		nvgStroke(vg);
	}

	void fillRect(const Vector2d &lr_, const Vector2d &LR_, const NVGcolor &COLOR) {
		Vector2f lr = _2nvg(lr_);
		Vector2f LR = _2nvg(LR_);
		Vector2f wh = LR - lr;
		// --
		nvgReset(vg);
		nvgBeginPath(vg);
		nvgRect(vg, lr.x(), lr.y(), wh.x(), wh.y());
		nvgFillColor(vg, COLOR);
		nvgFill(vg);
	}

	float CIRCLE_RADIUS = 4.f;
	void fillCircle(const Vector2d &s_, const NVGcolor &COLOR) {
		Vector2f s = _2nvg(s_);
		// --
		nvgReset(vg);
		nvgBeginPath(vg);
		nvgCircle(vg, s.x(), s.y(), CIRCLE_RADIUS);
		nvgFillColor(vg, COLOR);
		nvgFill(vg);
	}

	void drawVector(const Vector2d &s_, const Vector2d &F_, const NVGcolor &COLOR) {
		Vector2f s = _2nvg(s_);
		Vector2f t = _2nvg(s_ + F_);
		Vector2f st = t - s;
		Vector2f e = CIRCLE_RADIUS * Vector2f(-st.y(), st.x()).normalized();
		Vector2f sP = s + e;
		Vector2f sM = s - e;
		// --
		nvgReset(vg);
		nvgBeginPath(vg);
		nvgMoveTo(vg, t.x(), t.y());
		nvgLineTo(vg, sP.x(), sP.y());
		nvgLineTo(vg, sM.x(), sM.y());
		nvgLineTo(vg, t.x(), t.y());
		nvgFillColor(vg, COLOR);
		nvgFill(vg);
	}

};

int main(int, char**) {
	TranscriptionApp app(720, 720, "TranscriptionApp", !I_AM_HAVING_SCREEN_ISSUES ? 1.f : 2.f);
	app.run();
	return 0;
}
