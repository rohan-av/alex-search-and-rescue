
/**
*/
#include <ctime>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <raspicam/raspicam.h>
#include <vector>

using namespace std;

typedef struct Pixel{
    unsigned int red;
    unsigned int blue;
    unsigned int green;
} Pixel;

Pixel image[960][1280];

void parse(unsigned char *data){
    for (int i=0;i<960;i++){
        for(int j=0; j<1280; j++){
            Pixel p;
            p.red = (unsigned int)data[1280*3*i+3*j];
            p.blue = (unsigned int)data[1280*3*i+3*j+1];
            p.green = (unsigned int)data[1280*3*i+3*j+2];
            image[i][j] = p;
        }
    }
}

bool check_green(Pixel p){
    return (p.green > 100 && p.red < 60);
//    return (p.green > 60 && p.green > p.red + p.blue);
}

bool check_red(Pixel p){
    return (p.red > 110 && p.green < 60 && p.blue < 60);
}

vector<int> check_object(){
    vector<int>objects;
    objects.push_back(0);
    objects.push_back(0);
    for (int j=0; j<1280; j++){
        for (int i=0; i<960; i++){
            Pixel p = image[i][j];
            if (check_green(p)){
                objects[0] = 1;
                return objects;
            }
            else if (check_red(p)){
                objects[1] = 1;
                return objects;
            }    
        }
    }
}

void print_green(){
    for (int i = 0; i<960; i++){
        for (int j = 0; j<1280; j++){
            cout << i <<','<<j<<':'<<' '<< image[i][j].green << ' ';
        }
        cout << endl;
    }
}

int main ( int argc,char **argv ) {
    raspicam::RaspiCam Camera; //Camera object
    //Open camera
    Camera.setAWB_RB(1,0.5); 
    cout<<"Opening Camera..."<<endl;
    if ( !Camera.open()) {cerr<<"Error opening camera"<<endl;return -1;}
    //wait a while until camera stabilizes
    cout<<"Sleeping for 3 secs"<<endl;
    usleep(3*1000000);
    //capture
    Camera.grab();
    //allocate memory
    unsigned char *data=new unsigned char[  Camera.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB )];
    //extract the image in rgb format
    Camera.retrieve ( data,raspicam::RASPICAM_FORMAT_RGB );//get camera image
    //save
    /*
    cout << (unsigned int)data[2] << ' ' << (unsigned int)data[1] << ' '<<(unsigned int)data[0] <<endl; // first pixel RGB
    */
    for (int i=0; i<Camera.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB )/3; i++){
        unsigned char temp = data[i*3];
        data[i*3]=data[i*3+2];
        data[i*3+2]=temp;
    }
    /*
    unsigned int height = Camera.getHeight(); // 960
    unsigned int width = Camera.getWidth(); // 1280

    cout << height << ' ' << width << endl;
    */
    parse(data);
    vector<int> res = check_object();

    if (res[0]==1){
        cout << "Green object identified." << endl;
    }
    else if (res[1]==1){
        cout << "Red object identified." << endl;
    }

    std::ofstream outFile ( "raspicam_image.ppm",std::ios::binary );
    outFile<<"P6\n"<<Camera.getWidth() <<" "<<Camera.getHeight() <<" 255\n";
    outFile.write ( ( char* ) data, Camera.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB ) );
    cout<<"Image saved at raspicam_image.ppm"<<endl;
    //free resrources    
    delete data;
    return 0;
}
