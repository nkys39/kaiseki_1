
#include "slam_class.hpp"
#include <opencv2/opencv.hpp>

bool mapUpdate = false;
bool apply = true;

SLAM::SLAM(){	//ここは無視して良い
	_map_size = 3000;
	_map_size_x = 3000;
	_map_size_y = 3000;
    _err = -999999999;
    _map_rate = 50.0;
    _map_center = _map_size*0.5;
	_map_center_x = _map_size_x*0.5;
	_map_center_y = _map_size_y*0.5;
    _t2 = 500;

}
SLAM::~SLAM(){

}
// private
void SLAM::transformation_Rmatrix(struct robot_position *sa1, struct robot_position *sa2, double R[][3], double tv[])
{
    double roll = sa1->rangle/180.0*M_PI;
    double pitch = 0.0;
    double yaw = sa1->rangle/180.0*M_PI;
    
    
    tv[0] = sa1->real_x*0.001;
    tv[1] = sa2->real_x*0.001;
    tv[2] = sa1->real_y*0.001;
    
    R[0][0] = cos(roll)*cos(pitch), R[0][1] = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*sin(yaw), R[0][2] = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
    R[1][0] = sin(roll)*cos(pitch), R[1][1] = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw), R[1][2] = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
    R[2][0] = -sin(pitch),          R[2][1] = cos(pitch)*sin(yaw),                                R[2][2] = cos(pitch)*cos(yaw);
    
}

// Caluculate localization of the robot by using SSGA. Created by Yuichiro Toda.
double SLAM::fitcal_m(int i, struct robot_position *robot,int** &omap, int** &amap)
{
	int k;
	double h,h2;
	int hit=0,err=0;
	int num = 0;
	double x_1,y_1,x_3,y_3;
	int pt1_x,pt1_y;
	int hit2=0,err2=0,unce=0;
	double p=0.0,p2, p4 = 0.0;
	double x_c2,y_c2;
	double s1,c1;
	s1 = sin((robot->dangle+robot->gen_m[i][2])*M_PI/180.0);
	c1 = cos((robot->dangle+robot->gen_m[i][2])*M_PI/180.0);
	x_c2 = robot->real_x;
	y_c2 = robot->real_y;
	x_c2 += robot->gen_m[i][0]*c1 - robot->gen_m[i][1]*s1;
	y_c2 += robot->gen_m[i][1]*c1 + robot->gen_m[i][0]*s1;
	
	int ks,ke,skip;
    ks = 0;
    ke = (int)laser_x.size();
    skip = 2;
    p = 0.0;
	
	if(i>=0){
		for(k=ks;k<ke;k+=skip){
			
			x_1 = laser_x[k],y_1 = laser_y[k];
			if(x_1 != _err && y_1 != _err){
				x_3 = (double)x_1*c1 - (double)y_1*s1 + x_c2;
				y_3 = (double)x_1*s1 + (double)y_1*c1 + y_c2;
				pt1_x = _map_center_x - (int)x_3/_map_rate;
				pt1_y = _map_center_y - (int)y_3/_map_rate;
				
				if(pt1_x < 0 || pt1_x >= _map_size_x || pt1_y < 0 ||pt1_y > _map_size_y) continue;
				h = omap[pt1_x][pt1_y];
				h2 = amap[pt1_x][pt1_y];
				
				if( h2 > 0)
					hit++;
				else
					err++;
				
				if( random.cal_probability(h) > 0.3)
					hit2++;
				else if(random.cal_probability(h) < -0.3)
					err2++;
				else
					unce++;
				
				p4 += random.cal_probability(h);
				num++;
			}
		}
	}
	p2 = ((double)hit)/(double)(hit+err);
	p = ((double)hit2)/((double)hit2+(double)err2+(double)unce);
	p4 = (p*p2);

	//printf("fitcal_m = %4f\n", p4);

	return p4;
}

//matching with environment map and CAD map
double SLAM::fitcal_MIX(int i, struct robot_position* robot, int** &omap, int** &amap) {

	double fit_env = 0;
	double fit_cad = 0;
	double fit_mix = 0;
	double weight = 0;
	double model_ratio = 0;
	double dis_m, dis_g;

	fit_env = fitcal_m(i, robot, omap, amap);
	fit_cad = fitcal_m(i, robot, omap_CAD, amap_CAD);

	//printf("fit_env = %4f, fit_cad = %4f\n", fit_env, fit_cad);

	//model priority
	dis_m = sqrt((model[0] * model[0]) + (model[1] * model[1]));
	dis_g = sqrt((robot->gen_m[i][0] * robot->gen_m[i][0]) + (robot->gen_m[i][1] * robot->gen_m[i][1]));
	if(dis_m == 0){
		model_ratio = 0;
	}
	else{
		model_ratio = abs(dis_m - dis_g) / dis_m;
	}

	//printf("model_ratio = %4f\n", model_ratio);
	//

	weight = map_match_rate * (1 - model_ratio);
	fit_mix = weight * fit_env + (1 - weight) * fit_cad;

	//printf("fit_mix = %4f\n", fit_mix);

	return fit_mix;
}

int SLAM::g_max(struct robot_position *robot, double *best_fitness)
{
	int j;
	double max;
	for(int i=0;i<GANM;i++){
		if(i == 0){
			max = robot->fit_m[i];
			j = i;
		}else if(max < robot->fit_m[i]){
			max = robot->fit_m[i];
			j = i;
		}
	}
	*best_fitness = max;
	return j;
}

int SLAM::g_min(struct robot_position *robot, double *worst_fitness)
{
	int j;
	double min;
	for(int i=0;i<GANM;i++){
		if(i == 0){
			min = robot->fit_m[i];
			j = i;
		}else if(min > robot->fit_m[i]){
			min = robot->fit_m[i];
			j = i;
		}
	}
	*worst_fitness = min;
	return j;
}

void SLAM::mga_init(struct robot_position *robot, int** &omap, int** &amap)
{
	//kinematic model
	//base_scan is 19.4Hz = 0.05154639/s
	double dt = 0.05154639;
	model[0] = vel[0] * dt;
	model[1] = vel[1] * dt;
	model[2] = vel[2] * dt;

	for (int i = 0; i<GANM; i++)
	{
		if (i == 0) {
			robot->gen_m[i][0] = robot->gen_m[robot->best_gene][0];
			robot->gen_m[i][1] = robot->gen_m[robot->best_gene][1];
			robot->gen_m[i][2] = robot->gen_m[robot->best_gene][2];
			if (apply) {
				robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
			}
			else {
				robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
			}
			continue;
		}
		else if (i == 1) {
			robot->gen_m[i][0] = 0;
			robot->gen_m[i][1] = 0;
			robot->gen_m[i][2] = 0;
			if (apply) {
				robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
			}
			else {
				robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
			}
			continue;
		}
		else if (i == 2) {
			robot->gen_m[i][0] = -robot->gen_m[robot->best_gene][0];
			robot->gen_m[i][1] = -robot->gen_m[robot->best_gene][1];
			robot->gen_m[i][2] = -robot->gen_m[robot->best_gene][2];
			if (apply) {
				robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
			}
			else {
				robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
			}
			continue;
		}
		else { //init around kinematic model
			robot->gen_m[i][0] = model[0] + 40.0*random.rndn();
			robot->gen_m[i][1] = model[1] + 40.0*random.rndn();
			robot->gen_m[i][2] = model[2] + 40.0*random.rndn();
		}
		if (apply) {
			robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
		}
		else {
			robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
		}
	}
}


void SLAM::map_mating(struct robot_position *robot, int** &omap, int** &amap){
	
	int t=0,g,g2;
	int worst_gene;
 	double worst_fitness;
	double best_fitness;

	
	if (apply) {
		if (map_match_rate < 0.90) {//////////////////////////////////////////////////////////////////////////

			//printf("reset environment map\n");

			for (int i = 0; i < _map_size_y; i++) {
				for (int j = 0; j < _map_size_x; j++) {

					omap[i][j] = omap_CAD[i][j];
					amap[i][j] = amap_CAD[i][j];

				}
			}
			mapUpdate = true;
		}
	}
	
	
	mga_init(robot, omap, amap);
	
	while (t < _t2){
		double r,r2;
		robot->best_gene = g_max(robot, &best_fitness);
		worst_gene = g_min(robot, &worst_fitness);
		
		g = (int)(GANM*random.rnd());
		while(g == robot->best_gene)
			g = (int)(GANM*random.rnd());
		r2 = (10.0*(best_fitness - robot->fit_m[g])) / (best_fitness - worst_fitness + 0.01);
		g2 = robot->best_gene;
		for (int j = 0; j<GALM; j++) {
			r = random.rnd();
			//-------crossover-------------------------
			if (r < 0.7)
				robot->gen_m[worst_gene][j] = robot->gen_m[g2][j];
			else
				robot->gen_m[worst_gene][j] = (robot->gen_m[g][j] + robot->gen_m[g2][j])*random.rnd();

			//------mutation---------------------------
			if (random.rnd() < 0.7) {
				if (j != 2)
					robot->gen_m[worst_gene][j] += 10 * random.rndn_10()*(0.01 + r2*(double)(_t2 - t) / (double)_t2);
				else
					robot->gen_m[worst_gene][j] += random.rndn()*(0.01 + r2*(double)(_t2 - t) / (double)_t2);
			}
			else {
				if (j != 2)
					robot->gen_m[worst_gene][j] += 10.0*random.rndn_10()*random.rndn()*(0.01 + r2*(double)(_t2 - t) / (double)_t2);
				else
					robot->gen_m[worst_gene][j] += random.rndn()*random.rndn()*(0.01 + r2*(double)(_t2 - t) / (double)_t2);
			}
		}
		
		if (apply) {
			robot->fit_m[worst_gene] = fitcal_m(worst_gene, robot, omap, amap);
		}
		else {
			robot->fit_m[worst_gene] = fitcal_m(worst_gene, robot, omap, amap);
		}

		t++;
	}
	
	robot->best_gene = g_max(robot, &best_fitness);
//    printf("Best individual[%d] is %lf\n", robot->slam_count,best_fitness);
}

void SLAM::calc_mapvalue(int k, int l, int pt_x, int pt_y, int sflag, int** &omap, int** &amap)
{
	if(sflag == 1){
		if(k >= pt_x)
			omap[k][l]+=2;
		else
			omap[k][l]--;
	}else if(sflag == 2){
		if(k <= pt_x)
			omap[k][l]+=2;
		else
			omap[k][l]--;
	}else if(sflag == 3){
		if(l >= pt_y)
			omap[k][l]+=2;
		else
			omap[k][l]--;
	}else{
		if(l <= pt_y)
			omap[k][l]+=2;
		else
			omap[k][l]--;
	}
	
	if(random.cal_probability(amap[k][l]) > 0.85){
		if(omap[k][l] < 0 && amap[k][l] + omap[k][l] > 0){
			omap[k][l] += amap[k][l];
		}
	}
	
}


// public
// パラメータをセットする関数
void SLAM::set_param(int map_size_x, int map_size_y, int err, double map_rate,int t2){
	_map_size_x = map_size_x;
	_map_size_y = map_size_y;
	_err = err;
	_map_rate = map_rate;
	_map_center_x = map_size_x*0.5;
	_map_center_y = map_size_y*0.5;
	_t2 = t2;
}
// laser_dataをセットする関数
void SLAM::set_laser_data(long *lrf_range,int data_num,double angle_increment,double angle_min,double range_min,double range_max){
    
    laser_x.clear();
    laser_y.clear();
    double rad = angle_min;
    for(int i=0;i<data_num;i++){
        if(lrf_range[i] < range_min || lrf_range[i] > range_max){
            laser_x.push_back(_err);
            laser_y.push_back(_err);
        }else{
            laser_x.push_back(lrf_range[i]*cos(rad));
            laser_y.push_back(lrf_range[i]*sin(rad));
        }
        rad += angle_increment;
    }
}
// 自己位置追跡関数(tracking module)
void SLAM::slam(struct robot_position *robot, int** &omap, int** &amap){
    
	static double m_x, m_y;
	double s_x,s_y,s_r;
	static double pre_angle;


	
	pre_angle = robot->dangle;
	int i,j;
    if(robot->slam_count==0){		
		for(i=0;i<GANM+1;i++){
			robot->fit_m[i] = 0;
			for(j=0;j<GALM+1;j++)
				robot->gen_m[i][j] = 0;
		}
		robot->best_gene = 0;
	}
	
	if(robot->slam_count > 1){

		//compare environment map and CAD map
		if (apply) {
			compare_map(omap, amap);
		}
		

		map_mating(robot, omap, amap);
		
		m_x += robot->gen_m[robot->best_gene][0];
		m_y += robot->gen_m[robot->best_gene][1];
		robot->dangle += robot->gen_m[robot->best_gene][2];
		s_x = 0,s_y = 0,s_r=0;
		s_x = robot->gen_m[robot->best_gene][0];
		s_y = robot->gen_m[robot->best_gene][1];
		s_r = robot->gen_m[robot->best_gene][2];
		
        robot->real_x += s_x*cos(robot->dangle*M_PI/180) - s_y*sin(robot->dangle*M_PI/180);
        robot->real_y += s_y*cos(robot->dangle*M_PI/180) + s_x*sin(robot->dangle*M_PI/180);
        
        robot->rangle += robot->dangle - pre_angle;
        
        robot->map_x = _map_center_x - (int)robot->real_x/_map_rate; 
        robot->map_y = _map_center_y - (int)robot->real_y/_map_rate;
        
        double err_a;
        if(fabs(robot->rangle) > 180){
            err_a = fabs(robot->rangle) - 180.0;
            if(robot->rangle > 0){
                robot->rangle = -180.0;
                robot->rangle += err_a;
            }else{
                robot->rangle = 180.0;
                robot->rangle -= err_a;
            }
        }
	}
    
	robot->slam_count++;
}

void SLAM::map_building2(struct robot_position *robot, int** &omap, int** &amap){
    int i;
	int pt_x,pt_y,ptc_x,ptc_y;
	double x_3,y_3;
	
	int dx,dy;
	int j,e,k,l;
	double s1,c1;
	s1 = sin((double)robot->dangle*M_PI/180.0);
	c1 = cos((double)robot->dangle*M_PI/180.0);
	ptc_x = _map_center_x - (int)robot->real_x/_map_rate;
	ptc_y = _map_center_y - (int)robot->real_y/_map_rate;

	int ks,ke,skip;
    ks = 0;
    ke = (int)laser_x.size();
    skip = 2;
	
	for(j=ks;j<ke;j++){
		if(laser_x[j] != _err && laser_y[j] != _err){
			x_3 = (double)laser_x[j]*c1 - (double)laser_y[j]*s1 + (double)robot->real_x;
			y_3 = (double)laser_x[j]*s1 + (double)laser_y[j]*c1 + (double)robot->real_y;
			pt_x = _map_center_x - (int)x_3/_map_rate;
			pt_y = _map_center_y - (int)y_3/_map_rate;
            if(pt_x < 0 || pt_x > _map_size_x || pt_y < 0 || pt_y > _map_size_y) continue;
			amap[pt_x][pt_y]++;
			dx= (pt_x - ptc_x);
			dy= (pt_y - ptc_y);
			
			e=0;
			if(dx>=0&&dy>=0){
				if(dx>dy){
					for(l=ptc_y,k=ptc_x;k<=pt_x;k++){
						e+=dy;
						if(e > dx){
							e-=dx;
							l++;
						}
						
						calc_mapvalue(k, l, pt_x, pt_y, 1, omap, amap);
					}
				}else{
					for(l=ptc_y,k=ptc_x;l<=pt_y;l++){
						e+=dx;
						if(e > dy){
							e-=dy;
							k++;
						}
						
						calc_mapvalue(k, l, pt_x, pt_y, 3, omap, amap);
					}
				}
			}else if(dx>=0&&dy<=0){
				dy *= -1;
				if(dx>dy){
					for(l=ptc_y,k=ptc_x;k<=pt_x;k++){
						e+=dy;
						if(e > dx){
							e-=dx;
							l--;
						}
						
						calc_mapvalue(k, l, pt_x, pt_y, 1, omap, amap);
					}
				}else{
					for(l=ptc_y,k=ptc_x;l>=pt_y;l--){
						e+=dx;
						if(e > dy){
							e-=dy;
							k++;
						}
						
						calc_mapvalue(k, l, pt_x, pt_y, 4, omap, amap);
					}
				}
			}else if(dx<=0&&dy>=0){
				dx *= -1;
				if(dx>dy){
					for(l=ptc_y,k=ptc_x;k>=pt_x;k--){
						e+=dy;
						if(e > dx){
							e-=dx;
							l++;
						}
						
						calc_mapvalue(k, l, pt_x, pt_y, 2, omap, amap);
					}
				}else{
					for(l=ptc_y,k=ptc_x;l<=pt_y;l++){
						e+=dx;
						if(e > dy){
							e-=dy;
							k--;
						}
						
						calc_mapvalue(k, l, pt_x, pt_y, 3, omap, amap);
					}
				}
			}else{
				dx *= -1;
				dy *= -1;
				if(dx>dy){
					for(l=ptc_y,k=ptc_x;k>=pt_x;k--){
						e+=dy;
						if(e > dx){
							e-=dx;
							l--;
						}
						
						calc_mapvalue(k, l, pt_x, pt_y, 2, omap, amap);
					}
				}else{
					for(l=ptc_y,k=ptc_x;l>=pt_y;l--){
						e+=dx;
						if(e > dy){
							e-=dy;
							k--;
						}
						
						calc_mapvalue(k, l, pt_x, pt_y, 4, omap, amap);
					}
				}
			}
		}	
	}

	//save env info
	if (apply) {
		for (int i = 0; i < _map_size_y; i++) {
			for (int j = 0; j < _map_size_x; j++) {

				if (random.cal_probability(omap_CAD[i][j]) < 0.8 && random.cal_probability(omap[i][j]) >= 0.8 && amap[i][j] > 100) {

					omap_CAD[i][j] = omap[i][j];
					amap_CAD[i][j] = amap[i][j];

				}

			}
		}
	}
	//
}

void SLAM::compare_map(int** omap, int** amap) {

	int total = 0;
	int match = 0;
	double p_env = 0;
	double p_cad = 0;

	for (int i = 0; i < _map_size_y; i++) {
		for (int j = 0; j < _map_size_x; j++) {
			
			p_env = random.cal_probability(omap[i][j]);
			p_cad = random.cal_probability(omap_CAD[i][j]);

			if (p_env >= 0.8) {

				total++;
				if (p_cad >= 0.8) {
					match++;
				}

			}
		}
	}
	//printf("total = %d,  match = %d\n", total, match);

	map_match_rate = (double)match / (double)total;

	//printf("map_match_rate = %4f\n", map_match_rate);

}

// Initialize the input robot position. Created by Yuichiro Toda on 12/04/19.
void SLAM::initialize_RobotPosition(struct robot_position *robot, int dflag){
    if(dflag == 0){
		robot->real_x = -10*_map_rate;
		robot->real_y = -175*_map_rate;
		robot->rangle = 0.0;//degree
		robot->dangle = 0;
		robot->longitude = 0.0;
		robot->latitude = 0.0;
		robot->map_x = _map_center_x - 10.0;
		robot->map_y = _map_center_y - 175.0;
		robot->slam_count = 0;
	}else{
		robot->real_x = _err;
		robot->real_y = _err;
		robot->rangle = 0;
		robot->dangle = 0;
		robot->map_x = 0;
		robot->map_y = 0;
		robot->longitude = 0.0;
		robot->latitude = 0.0;
		robot->slam_count = 0;
	}
}

// void SLAM::calcRobotModel(double model[3]){
// 	static double wheeldia = 125.0; //[mm]
// 	static double wheelwidth = 350.0; //[mm]
// 	static double time = 0.025; //[s]
// 	static double lidarwheeldist = 0.0; //[mm]

// 	double vL, vR; //speed(odom) [mm/s]

// 	double v = (vL + vR) * 0.5; //speed
// 	double w = (vR - vL) / wheelwidth; //rotation speed
// 	double l = v / w;

// 	double dx, dy, dr; //movement
// 	if(abs(vR - vL) < 1e-10){
// 		dx = 0.0;
// 		dy = v * dt;
// 		dr = 0.0;
// 	}
// 	else{
// 		dx = l * cos(w * dt) - l * cos(0) + h * sin(w * dt);
// 		dy = l * sin(w * dt) - l * sin(0) + h * cos(w * dt);
// 		dr = w * dt * 180 / PI();
// 	}

// 	model[0] = -dx;
// 	model[1] = dy;
// 	model[2] = dr;

// }

