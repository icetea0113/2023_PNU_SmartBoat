#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int main()
{
    ifstream gpsData;
    gpsData.open("/dev/serial0");

    if (!gpsData)
    {
        cout << "Error opening GPS data stream" << endl;
        return 1;
    }

    string line;
    while (getline(gpsData, line))
    {
        if (line.substr(0, 6) == "$GPGGA")
        {
            int commaPos = line.find(",");
            int latStart = commaPos + 1;
            int latEnd = line.find(",", latStart);
            string lat = line.substr(latStart, latEnd - latStart);

            int lonStart = latEnd + 1;
            int lonEnd = line.find(",", lonStart);
            string lon = line.substr(lonStart, lonEnd - lonStart);

            cout << "Latitude: " << lat << " Longitude: " << lon << endl;
        }
    }

    gpsData.close();
    return 0;
}