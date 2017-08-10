#include "wheretoGo.h"



using namespace std;



int main(){

    wheretoGo Beta;
    
    if(Beta.foundPaths() == 1){

    cout << "No Paths Found-Generating\n";
    Beta.generatePaths();
    cout << "Paths generated\n";

    }

}


