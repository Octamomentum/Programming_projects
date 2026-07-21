#include "Workshop.h"
#include "Car.h"

int main(){

Workshop ws;
Car* car1 = new Car("ABC-123", 7000, 0.75, false);
Car* car2 = new Car("DEF-456", 12000, 3.5, true);

ws.addVehicleToRepair(car1);
ws.addVehicleToRepair(car2);

ws.ServiceForAllVehicles();


}