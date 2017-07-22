#include "wheretoGo.h"



using namespace std;



int main(){

    wheretoGo Beta;
    
    if(Beta.foundPaths() == 1){

    cout << "No Paths Found\nGenerating";
    generatePaths();
    cout << "Paths generated\n";
    
    }
    
}


