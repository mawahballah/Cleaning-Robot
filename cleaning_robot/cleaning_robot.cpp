#include<unordered_set>
#include<vector>
#include<iostream>
#include<algorithm>
#include<cstdlib>
#include<boost/functional/hash.hpp>
#include<boost/filesystem.hpp>
#include<json.hpp>
#include<fstream>
#include<string>
#include <iomanip>
#include<fifo_map.hpp>

// A workaround to give to use fifo_map as map, just ignoring the 'less' compare
// to stop the json library from sorting the objects ascendingly
template<class K, class V, class dummy_compare, class A>
using my_workaround_fifo_map = fifo_map<K, V, fifo_map_compare<K>, A>;
using my_json = nlohmann::basic_json<my_workaround_fifo_map>;
enum status {
	SUCCESS,
	NO_BATTERY,
	BLOCKED
};
//the robot itself
class Robot {
private:	
	std::pair<int, int>position;
	std::vector<char> orientations; // has all orientations
	int battery;
	int orientationIndex; // not to search in the orientation vector everytime
public:
	Robot(int positionX, int positionY, char orient, int batterylevel)
	{
		position = std::make_pair(positionX, positionY);
		orientations = { 'N','E','S','W' };
		orientationIndex = std::find(orientations.begin(), orientations.end(), orient) - orientations.begin();
		battery = batterylevel;
	}		
	bool turnLeft() {
		if (getBattery() - 1 < 0)
			return false; // no battery
		orientationIndex = (orientationIndex + 3) % 4;
		reduceBattery(1);
		return true;
	}
	bool turnRight()
	{
		if (getBattery() - 1 < 0)
			return false;// no battery
		orientationIndex = (orientationIndex + 1) % 4;
		reduceBattery(1);
		return true;
	}
	bool clean()
	{
		if (getBattery() - 5 < 0)
			return false;// not battery
		reduceBattery(5);
		return true;
	}
	void changePosition(int newX, int newY) {
		position = std::make_pair(newX, newY);
	}
	std::pair<int, int> getPosition()
	{
		return position;
	}
	char getOrientation() {
		return orientations[orientationIndex];
	}
	int getBattery() { return battery; }
	void reduceBattery(int value)
	{
		battery -= value;
	}
};
//class that has an object of the robot and the map
//responsible for movements on the map and keeps track of the robot's visited
//and cleaned co-ordinates
class RobotMoves {
	Robot *myrobot;
	std::vector<std::vector<std::string>> mymap;
	std::unordered_set<std::pair<int, int>, boost::hash<std::pair<int, int>>>visited, cleaned;//boost supports having pairs in unordered_set
public:	
	RobotMoves(int x, int y, char orient, int battery, std::vector<std::vector<std::string>>m) {
		myrobot = new Robot(x, y, orient, battery);
		visited.insert(this->getPosition());
		mymap = m;
	}
	//copy constructor
	RobotMoves(const RobotMoves &obj)
	{
		this->myrobot = new Robot(*obj.myrobot);
		this->mymap = obj.mymap;
		this->visited = obj.visited;
		this->cleaned=obj.cleaned;
	}
	//Assignment operator
	RobotMoves& operator=(const RobotMoves& otherRobotMoves)
	{
		if (this != &otherRobotMoves)
		{
			this->myrobot = otherRobotMoves.myrobot;
			this->mymap = otherRobotMoves.mymap;
			this->visited = otherRobotMoves.visited;
			this->cleaned = otherRobotMoves.cleaned;
		}
		return *this;
	}
	~RobotMoves() {
		delete myrobot;
	}	
	bool applyAdvance(std::pair<int, int>position_change, int used_battery)
	{
		std::pair<int, int> currentposition = myrobot->getPosition();
		if (validCoordinates(currentposition.first + position_change.first, currentposition.second + position_change.second))
		{
			myrobot->changePosition(currentposition.first + position_change.first, currentposition.second + position_change.second);
			myrobot->reduceBattery(used_battery);
			visited.insert(this->getPosition());
			return true;
		}
		return false;
	}
	status back()
	{
		if (myrobot->getBattery() - 3 < 0)
			return status::NO_BATTERY;
		char orientation = myrobot->getOrientation();
		myrobot->reduceBattery(3);
		switch (orientation)
		{
		case 'N':
			if (applyAdvance({ 1,0 }, 0))
				return status::SUCCESS;
		case 'S':
			if (applyAdvance({ -1,0 }, 0))
				return status::SUCCESS;
		case 'E':
			if (applyAdvance({ 0,-1 }, 0))
				return status::SUCCESS;
		case 'W':
			if (applyAdvance({ 0,1 }, 0))
				return status::SUCCESS;
		default:
			break;
		}
		return status::BLOCKED;
	}
	status advance() {
		if (myrobot->getBattery() - 2 < 0)
			return status::NO_BATTERY;
		char orientation = myrobot->getOrientation();
		switch (orientation)
		{
		case 'N':
			if (applyAdvance({ -1,0 }, 2))
				return status::SUCCESS;
			break;
		case 'S':
			if (applyAdvance({ 1,0 }, 2))
				return status::SUCCESS;
			break;
		case 'E':
			if (applyAdvance({ 0,1 }, 2))
				return status::SUCCESS;
			break;
		case 'W':
			if (applyAdvance({ 0,-1 }, 2))
				return status::SUCCESS;
			break;
		default:
			break;
		}
		return status::BLOCKED;
	}
	status backoffStrategy4()
	{
		myrobot->reduceBattery(2);
		if (!myrobot->turnRight() || back() == status::NO_BATTERY || !myrobot->turnRight())
			return status::NO_BATTERY;
		return advance();
	}
	status backoffStrategy35() {
		myrobot->reduceBattery(2);
		if (!myrobot->turnLeft() || !myrobot->turnLeft())
			return status::NO_BATTERY;
		return advance();
	}
	status backoffStrategy2() {
		myrobot->reduceBattery(2);
		if (!myrobot->turnLeft() || back() == status::NO_BATTERY || !myrobot->turnRight())
			return status::NO_BATTERY;
		return advance();
	}
	status backoffStrategy1()
	{
		myrobot->reduceBattery(2);
		if (!myrobot->turnRight())
			return status::NO_BATTERY;
		return advance();
	}
	//backoff strategy controller
	bool backoffStrategy() {
		status currentStatus;
		if ((currentStatus = backoffStrategy1()) == status::BLOCKED &&
			(currentStatus = backoffStrategy2()) == status::BLOCKED &&
			(currentStatus = backoffStrategy35()) == status::BLOCKED &&
			(currentStatus = backoffStrategy4()) == status::BLOCKED &&
			(currentStatus = backoffStrategy35()) == status::BLOCKED)
		{
			myrobot->reduceBattery(2);
			return false;
		}
		if (currentStatus == status::NO_BATTERY)// at anypoint the program stops if no battery
			return false;
		return true;
	}
	//first try to advance without backoff strategies 
	bool advanceTrial()
	{
		status result = advance();
		if (result == status::NO_BATTERY)
			return false;
		else if (result == status::BLOCKED)
			return backoffStrategy(); //only if blocked
		return true;
	}
	bool turnLeft() {
		return myrobot->turnLeft();
	}
	bool turnRight() {
		return myrobot->turnRight();
	}
	bool clean() {
		if (myrobot->clean())
		{
			cleaned.insert(this->getPosition());
			return true;
		}
		return false;
	}
	//check if the co-ordinates of the new location are valid
	bool validCoordinates(int x, int y)
	{
		if (x >= mymap.size() || x < 0 || y >= mymap[0].size() || y < 0 || mymap[x][y] == "null" || mymap[x][y] == "C")
			return false;
		return true;
	}
	std::pair<int, int>getPosition() {
		return myrobot->getPosition();
	}
	int getBattery() {
		return myrobot->getBattery();
	}
	char getOrientation() {
		return myrobot->getOrientation();
	}
	//write the visited co-ordinates in the json file
	void writeVisited(my_json& outputFile)
	{
		nlohmann::json arr;
		for (auto it = visited.begin(); it != visited.end(); ++it)
		{
			nlohmann::json coordinate;
			coordinate["X"] = it->second;
			coordinate["Y"] = it->first;
			arr.push_back(coordinate);
		}
		std::sort(arr.begin(), arr.end());
		outputFile["visited"] = arr;
	}
	//write the cleaned co-ordinates in the json file
	void writeCleaned(my_json& outputFile)
	{
		nlohmann::json arr;
		for (auto it = cleaned.begin(); it != cleaned.end(); ++it)
		{
			nlohmann::json coordinate;
			coordinate["X"] = it->second;
			coordinate["Y"] = it->first;
			arr.push_back(coordinate);
		}
		std::sort(arr.begin(), arr.end());
		outputFile["cleaned"] = arr;
	}
	void writeRobot(my_json& outputFile)
	{
		this->writeVisited(outputFile);
		this->writeCleaned(outputFile);
		outputFile["final"]["X"] = this->getPosition().second;
		outputFile["final"]["Y"] = this->getPosition().first;
		outputFile["final"]["facing"] = std::string(1, this->getOrientation());
		outputFile["battery"] = this->getBattery();
	}
};
RobotMoves *readJson(nlohmann::json &commands, std::string fileName)
{
	std::ifstream input(fileName);
	nlohmann::json jComplete = nlohmann::json::parse(input);
	nlohmann::json jMap = jComplete["map"];
	commands = jComplete["commands"];
	nlohmann::json jBattery = jComplete["battery"];
	nlohmann::json jStart = jComplete["start"];
	auto it = jStart.find("facing");
	std::string facing = *it;
	RobotMoves *robotMoves= new RobotMoves(jStart["Y"], jStart["X"], facing[0], jBattery, jMap);
	return robotMoves;
}
void processCommands(nlohmann::json &commands, RobotMoves*robotMoves)
{

	for (int i = 0; i < commands.size(); i++)
	{
		if (commands[i] == "A")
		{
			if (!robotMoves->advanceTrial())
				break;
		}
		else if (commands[i] == "C")
		{
			if (!robotMoves->clean())
				break;
		}
		else if (commands[i] == "TL")
		{
			if (!robotMoves->turnLeft())
				break;
		}
		else
		{
			if (!robotMoves->turnRight())
				break;
		}
	}

}
void writeJsonFile(RobotMoves*robotMoves, std::string fileName)
{
	std::ofstream output(fileName);
	my_json jsonOutput;
	robotMoves->writeRobot(jsonOutput);
	output <<  std::setw(2) << jsonOutput;
}
void helpInputFile()
{
	std::cout << "command line arguments:\n";
	std::cout << "input file doesn't exist";
}
void helpExtension()
{
	std::cout << "command line arguments:\n";
	std::cout << "input/output file isn't a json file";
}
bool checkValid(int argc, char*argv[])
{
	if (argc != 3)
		return false;
	if (!boost::filesystem::exists(argv[1])) //check if input file exists
	{
		helpInputFile();
		return false;
	}
	if (boost::filesystem::extension(argv[1]) != ".json" || boost::filesystem::extension(argv[2]) != ".json") // check if extension is json
	{
		helpExtension();
		return false;
	}

	return true;
}

int main(int argc, char*argv[])
{
	if (checkValid(argc, argv)) // verify the validity of the input
	{
		nlohmann::json jCommands;
		RobotMoves *robotMoves = readJson(jCommands, argv[1]); //read data from json file and create RobotMoves object
		processCommands(jCommands, robotMoves); //process the commands
		writeJsonFile(robotMoves, argv[2]); //write the output in json file
	}
	return 0;
}