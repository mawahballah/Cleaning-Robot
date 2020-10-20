# Cleaning Robot

Using Boost, a code for an automated robot is developed that can clean the house without any manual intervention

## Getting Started

Download the files in this repository

### Prerequisites

- Setup Boost, a good and easy way to do it, is by following the steps [here](https://stackoverflow.com/questions/2629421/how-to-use-boost-in-visual-studio-2010/2655683#2655683)

- Add json.hpp to your includes folder (can be found [here](https://github.com/nlohmann/json/tree/develop/single_include/nlohmann))

- Add fifo_map.hpp to your includes folder (can be found [here](https://github.com/nlohmann/fifo_map/tree/master/src))

- In Visual Studio go to Debug -> cleaning_robot Properties -> Debugging -> Command Arguments and add the following

```
test1.json test1_result.json
```


## Testing

Build the project in Visual Studio, open cmd and change directory to the Debug folder and write the following in the command line

```
cleaning_robot test1.json test1_result.json
```
test1.json is the input file and test1_result.json is the output file (you can do the same with other test json files).

## Built With

* [Boost](https://www.boost.org/) 

* [JSON for Modern C++](https://github.com/nlohmann/json) 

## Authors

* **Mohamed Wahballah** - *Initial work* - [mawahballah](https://github.com/mawahballah)
