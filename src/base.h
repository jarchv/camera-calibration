int countFrame      = 0;
int T_width;
int T_height;
float temp_time;
float time_avr      = 0;
float time_elapsed;

int predict;
float ground_truth = 20;
float accuracy; 
float erros=0.0;

int CAPTURING  = 0;
int CALIBRATED = 1;
int RECONSTRUCTION = 2;
int mode       = 0;

float BIAS = 200;
int   IMAGES_TO_CALIBRATE = 8;

std::string VISUALIZE_PATH = "visualize/";
std::string TXT_PATH       = "txt/";