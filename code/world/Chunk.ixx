export module Chunk;

import std;
import util;
import constVar;
import TileData;

export class Chunk
{
private:
	std::array<std::array<TileData, CHUNK_SIZE_Y>, CHUNK_SIZE_X> singleTile;
	chunkFlag flag = chunkFlag::none;
	weatherFlag chunkWeather = weatherFlag::sunny;
	
	std::unordered_set<Monster*> chunkMonsterSet;
    std::unordered_set<Vehicle*> chunkVehicleSet;
	std::unordered_set<Prop*> chunkPropSet;
	std::unordered_set<ItemStack*> chunkStackSet;
public:
	Chunk(chunkFlag input)
	{
		//prt(lowCol::green, L"Chunk : 생성자가 호출되었습니다..\n");
		chunkLoad(input);
	}
	~Chunk()
	{
		//prt(lowCol::green, L"Chunk : 소멸자가 호출되었습니다..\n");
	}
	TileData& getChunkTile(int x, int y) { return singleTile[y][x]; }

	void chunkLoad(chunkFlag inputChunk)
	{
		for (int x = 0; x < CHUNK_SIZE_X; x++)
		{
			for (int y = 0; y < CHUNK_SIZE_Y; y++)
			{
				singleTile[x][y].randomVal = randomRange(0, 65535);
			}
		}

		if (inputChunk == chunkFlag::seawater)
		{
			//prt(lowCol::blue, L"Chunk : 이 청크는 해수 타일이다..\n");
			for (int x = 0; x < CHUNK_SIZE_X; x++)
			{
				for (int y = 0; y < CHUNK_SIZE_Y; y++)
				{
					singleTile[x][y].floor = itemRefCode::deepSeaWater;
				}
			}
		}
		else if (inputChunk == chunkFlag::none)
		{
			//prt(lowCol::blue, L"Chunk : 이 청크는 해수 타일이다..\n");
			for (int x = 0; x < CHUNK_SIZE_X; x++)
			{
				for (int y = 0; y < CHUNK_SIZE_Y; y++)
				{
					singleTile[x][y].floor = 0;
				}
			}
		}
		else if (inputChunk == chunkFlag::underground)
		{
			for (int x = 0; x < CHUNK_SIZE_X; x++)
			{
				for (int y = 0; y < CHUNK_SIZE_Y; y++)
				{
					singleTile[x][y].floor = 109;
					singleTile[x][y].setWall(302);
				}
			}
		}
		else if (inputChunk == chunkFlag::meadow)
		{
			for (int x = 0; x < CHUNK_SIZE_X; x++)
			{
				for (int y = 0; y < CHUNK_SIZE_Y; y++)
				{
					singleTile[x][y].floor = 220;
				}
			}
		}
		else if (inputChunk == chunkFlag::dirt)
		{
			for (int x = 0; x < CHUNK_SIZE_X; x++)
			{
				for (int y = 0; y < CHUNK_SIZE_Y; y++)
				{
					singleTile[x][y].floor = 109;
				}
			}
		}
		else
		{
			//prt(lowCol::blue, L"Chunk : 이 청크는 해수 타일이다..\n");
			for (int x = 0; x < CHUNK_SIZE_X; x++)
			{
				for (int y = 0; y < CHUNK_SIZE_Y; y++)
				{
					singleTile[x][y].floor = itemRefCode::grass;
				}
			}
		}
		flag = inputChunk;
	}

	chunkFlag getFlag()
	{
		return flag;
	}
	void setFlag(chunkFlag input)
	{
		flag = input;
	}


	Vehicle* getChunkVehiclePos(int x, int y) { return (Vehicle*)getChunkTile(x, y).VehiclePtr; }
	void setChunkVehiclePos(int x, int y, Vehicle* inputPtr) { prt(lowCol::green, L"Chunk : %d,%d에 Vehicle %p를 배정했다.\n", x, y, inputPtr); getChunkTile(x, y).VehiclePtr = inputPtr; }

	std::vector<Entity*> getChunkEntityList()
	{
		std::vector<Entity*> entityList;
		for (int x = 0; x < CHUNK_SIZE_X; x++)
		{
			for (int y = 0; y < CHUNK_SIZE_Y; y++)
			{
				if (getChunkTile(x, y).EntityPtr != nullptr) { entityList.push_back(getChunkTile(x, y).EntityPtr.get()); }
			}
		}
		return entityList;
	}

	std::vector<Vehicle*> getChunkVehicleList()
	{
		std::vector<Vehicle*> VehicleList;
		for (int x = 0; x < CHUNK_SIZE_X; x++)
		{
			for (int y = 0; y < CHUNK_SIZE_Y; y++)
			{
				if (getChunkTile(x, y).VehiclePtr != nullptr)
				{
					VehicleList.push_back((Vehicle*)getChunkTile(x, y).VehiclePtr);
				}
			}
		}
		return VehicleList;
	}

	std::vector<Prop*> getChunkPropList()
	{
		std::vector<Prop*> propList;
		for (int x = 0; x < CHUNK_SIZE_X; x++)
		{
			for (int y = 0; y < CHUNK_SIZE_Y; y++)
			{
				if (getChunkTile(x, y).PropPtr != nullptr)
				{
					propList.push_back(getChunkTile(x, y).PropPtr.get());
				}
			}
		}
		return propList;
    }

	std::vector<ItemStack*>& getChunkStackVec()
	{
		std::vector<ItemStack*> stackVec;
		for (int x = 0; x < CHUNK_SIZE_X; x++)
		{
			for (int y = 0; y < CHUNK_SIZE_Y; y++)
			{
				if (getChunkTile(x, y).ItemStackPtr != nullptr)
				{
					stackVec.push_back(getChunkTile(x, y).ItemStackPtr.get());
				}
			}
		}
		return stackVec;
	}

	weatherFlag getWeather() {
		return chunkWeather;
	}

	void setWeather(weatherFlag input) {
		chunkWeather = input;
	}

	chunkFlag getChunkFlag() {
		return flag;
	}


	//Monster 관련 함수들
	void addMonster(Monster* monster)
	{
		if (monster != nullptr) chunkMonsterSet.insert(monster);
	}
	const std::unordered_set<Monster*>& getMonsterSet() const { return chunkMonsterSet; }
	bool eraseMonster(Monster* monster) { return chunkMonsterSet.erase(monster) > 0; }

	// Vehicle 관련 함수들
	void addVehicle(Vehicle* vehicle)
	{
		if (vehicle != nullptr) chunkVehicleSet.insert(vehicle);
	}
	const std::unordered_set<Vehicle*>& getVehicleSet() const { return chunkVehicleSet; }
	bool eraseVehicle(Vehicle* vehicle) { return chunkVehicleSet.erase(vehicle) > 0; }

	// Prop 관련 함수들
	void addProp(Prop* prop)
	{
		if (prop != nullptr) chunkPropSet.insert(prop);
	}
	const std::unordered_set<Prop*>& getPropSet() const { return chunkPropSet; }
	bool eraseProp(Prop* prop) { return chunkPropSet.erase(prop) > 0; }
};