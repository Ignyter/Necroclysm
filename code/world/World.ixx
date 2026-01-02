#include <SDL3_image/SDL_image.h>

export module World;

import std;
import util;
import Chunk;
import Mapmaker;
import constVar;
import TileData;

export class World
{
private:
	std::unordered_map<Point3, Chunk*, Point3::Hash> chunkPtr;
	std::vector<Chunk*> activeChunk;
	std::unordered_set<Point3, Point3::Hash> isSectorCreated;

public:
	static World* ins()//싱글톤 함수
	{
		static World* ptr = new World();
		return ptr;
	}
	World()
	{
		const int baseRange = 4;

		for (int y = -baseRange; y <= baseRange; y++)
		{
			for (int x = -baseRange; x <= baseRange; x++)
			{
				chunkPtr[{x, y, 0}] = new Chunk(chunkFlag::seawater);
			}
		}

		for (int y = -baseRange; y <= baseRange; y++)
		{
			for (int x = -baseRange; x <= baseRange; x++)
			{
				chunkPtr[{x, y, 1}] = new Chunk(chunkFlag::none);
			}
		}

		for (int y = -baseRange; y <= baseRange; y++)
		{
			for (int x = -baseRange; x <= baseRange; x++)
			{
				chunkPtr[{x, y, -1}] = new Chunk(chunkFlag::underground);
			}
		}
	}

	TileData& getTile(int x, int y, int z)
	{
		int chunkX, chunkY;
		changeToChunkCoord(x, y, chunkX, chunkY);
		int localX = x - (chunkX * CHUNK_SIZE_X);
		int localY = y - (chunkY * CHUNK_SIZE_Y);
		return chunkPtr[{chunkX, chunkY, z}]->getChunkTile(localX, localY);
	}
	TileData& getTile(Point3 inputCoor)
	{
		return getTile(inputCoor.x, inputCoor.y, inputCoor.z);
	}

	void createChunk(int chunkX, int chunkY, int chunkZ)
	{
		chunkFlag inputFlag = chunkFlag::seawater;
		if (chunkZ > 0) inputFlag = chunkFlag::none;
		else if (chunkZ < 0) inputFlag = chunkFlag::underground;

		if (Mapmaker::ins()->isEmptyProphecy(chunkX, chunkY, chunkZ) == false) inputFlag = Mapmaker::ins()->getProphecy(chunkX, chunkY, chunkZ);

		chunkPtr[{chunkX, chunkY, chunkZ}] = new Chunk(inputFlag);
	}
	bool existChunk(int chunkX, int chunkY, int chunkZ)
	{
		if (chunkPtr.find({ chunkX,chunkY,chunkZ }) != chunkPtr.end()) return true;
		else return false;
	}
	void changeToChunkCoord(int x, int y, int& chunkX, int& chunkY)
	{
		chunkX = (x >= 0)
			? (x / CHUNK_SIZE_X)
			: ((x - (CHUNK_SIZE_X - 1)) / CHUNK_SIZE_X);

		chunkY = (y >= 0)
			? (y / CHUNK_SIZE_Y)
			: ((y - (CHUNK_SIZE_Y - 1)) / CHUNK_SIZE_Y);

	}
	Point2 changeToChunkCoord(int x, int y)
	{
		int chunkX = (x >= 0)
			? (x / CHUNK_SIZE_X)
			: ((x - (CHUNK_SIZE_X - 1)) / CHUNK_SIZE_X);

		int chunkY = (y >= 0)
			? (y / CHUNK_SIZE_Y)
			: ((y - (CHUNK_SIZE_Y - 1)) / CHUNK_SIZE_Y);

		return Point2{ chunkX,chunkY };
	}
	void activate(int x, int y, int z)
	{
		activeChunk.push_back(chunkPtr[{x, y, z}]);
	}
	void deactivate()
	{
		activeChunk.clear();
	}

	std::vector<Chunk*>& getActiveChunk()
	{
		return activeChunk;
	}

	std::unordered_set<Monster*> getActiveMonsterSet()
	{
		std::unordered_set<Monster*> totalMonsterSet;
		const std::unordered_set<Monster*>* chunkMonsterSet;
		for (int i = 0; i < activeChunk.size(); i++)
		{
			chunkMonsterSet = &(activeChunk[i]->getMonsterSet());
			for (auto monster : *chunkMonsterSet)
			{
				totalMonsterSet.insert(monster);
			}
		}
		return totalMonsterSet;
	}

	std::unordered_set<Vehicle*> getActiveVehicleSet()
	{
		std::unordered_set<Vehicle*> totalVehicleSet;
		const std::unordered_set<Vehicle*>* chunkVehicleSet;
		for (int i = 0; i < activeChunk.size(); i++)
		{
			chunkVehicleSet = &(activeChunk[i]->getVehicleSet());
			for (auto vehicle : *chunkVehicleSet)
			{
				totalVehicleSet.insert(vehicle);
			}
		}
		return totalVehicleSet;
	}

	std::unordered_set<Prop*> getActivePropSet()
	{
		std::unordered_set<Prop*> totalPropSet;
		const std::unordered_set<Prop*>* chunkPropSet;
		for (int i = 0; i < activeChunk.size(); i++)
		{
			chunkPropSet = &(activeChunk[i]->getPropSet());
			for (auto prop : *chunkPropSet)
			{
				totalPropSet.insert(prop);
			}
		}
		return totalPropSet;
	}

	std::vector<ItemStack*>& getActiveStackVec()
	{
		std::vector<ItemStack*> totalStackVec;
		for (int i = 0; i < activeChunk.size(); i++)
		{
			const std::vector<ItemStack*>& chunkStackVec = activeChunk[i]->getChunkStackVec();
			for (auto stack : chunkStackVec)
			{
				totalStackVec.push_back(stack);
			}
		}
		return totalStackVec;
	}

	//섹터 관련
	Point2 changeToSectorCoord(int inputGridX, int inputGridY)
	{
		//-1200 ~ -801 : -3
		//-800 ~ -401 : -2
		//-400~ -1 : -1
		//0~399 : 0
		//400~799 : 1
		//800~1199 : 2

		int sectorX, sectorY;
		int chunkX, chunkY;

		//그리드 좌표를 청크로 변환
		changeToChunkCoord(inputGridX, inputGridY, chunkX, chunkY);

		//청크 좌표를 섹터 좌표로 변환
		if (chunkX > 0) sectorX = chunkX / SECTOR_SIZE;
		else sectorX = (chunkX + 1) / SECTOR_SIZE - 1;

		if (chunkY > 0) sectorY = chunkY / SECTOR_SIZE;
		else sectorY = (chunkY + 1) / SECTOR_SIZE - 1;

		return { sectorX,sectorY };
	}

	bool isEmptySector(int sectorX, int sectorY, int sectorZ)
	{
		if (isSectorCreated.find({ sectorX,sectorY,sectorZ }) == isSectorCreated.end()) return true;
		else return false;
	}

	void createSector(int sectorX, int sectorY, int sectorZ)
	{
		if (sectorZ == 0)
		{
			if ((sectorY <= 26 && sectorY >= -27) && (sectorX <= 53 && sectorX >= -54))
			{
				std::string filePath = "map/worldSector-";
				int number = 2971 + sectorX + 108 * sectorY;
				if (number < 100) filePath += "0";
				filePath += std::to_string(number);
				filePath += ".png";
				std::wstring wPath(filePath.begin(), filePath.end());
				//std::wprintf(L"[World] Sector : %ls의 파일을 읽어내었다.\n", wPath.c_str());
				SDL_Surface* refSector = IMG_Load(filePath.c_str());

				if (!refSector)   // 디버깅: 로드 실패 이유 출력
				{
					// SDL_GetError() → std::string → std::wstring
					std::wstring sdlErr = stringToWstring(std::string(SDL_GetError()));

					// 실행 중의 작업 디렉터리 (char→wstring)
					std::wstring cwd = std::filesystem::current_path().wstring();

					std::wstring msg =
						L"IMG_Load 실패\n"
						L"  SDL_GetError : " + sdlErr +
						L"\n  시도한 경로   : " + wPath +
						L"\n  현재 CWD      : " + cwd + L'\n';

					prt(L"%ls", msg.c_str());               // 콘솔/디버그 출력
				}

				errorBox(refSector == NULL, L"섹터의 파일 읽기가 실패하였습니다. :" + std::to_wstring(sectorX) + L"," + std::to_wstring(sectorY) + L"," + std::to_wstring(sectorZ));
				Uint32* pixels = (Uint32*)refSector->pixels;


				for (int x = 0; x < 400; x++)
				{
					for (int y = 0; y < 400; y++)
					{
						chunkFlag targetFlag = chunkFlag::none;

						Uint32 pixel = pixels[(y * refSector->w) + x];
						SDL_Color pixelCol;
						SDL_GetRGB(pixel,
							SDL_GetPixelFormatDetails(refSector->format),
							SDL_GetSurfacePalette(refSector),
							&pixelCol.r, &pixelCol.g, &pixelCol.b);

						auto isSameCol = [](SDL_Color col1, SDL_Color col2)->bool
							{
								if (col1.r == col2.r)
								{
									if (col1.g == col2.g)
									{
										if (col1.b == col2.b)
										{
											return true;
										}
									}
								}

								return false;
							};

						if (isSameCol(pixelCol, chunkCol::seawater)) targetFlag = chunkFlag::seawater;
						else if (isSameCol(pixelCol, chunkCol::land)) targetFlag = chunkFlag::dirt;
						else if (isSameCol(pixelCol, chunkCol::city)) targetFlag = chunkFlag::dirt;
						else if (isSameCol(pixelCol, chunkCol::river)) targetFlag = chunkFlag::seawater;

						//섹터 좌표로 청크 좌표 구하기
						int chunkOriginX, chunkOriginY;
						chunkOriginX = 400 * sectorX;
						chunkOriginY = 400 * sectorY;

						Mapmaker::ins()->addProphecy(chunkOriginX + x, chunkOriginY + y, sectorZ, targetFlag);
					}
				}

				SDL_DestroySurface(refSector);
			}
		}

		isSectorCreated.insert({ sectorX,sectorY,sectorZ });
	}

	chunkFlag getChunkFlag(int chunkX, int chunkY, int chunkZ)
	{
		return chunkPtr[{chunkX, chunkY, chunkZ}]->getChunkFlag();
	}
	weatherFlag getChunkWeather(int chunkX, int chunkY, int chunkZ)
	{
		return chunkPtr[{chunkX, chunkY, chunkZ}]->getWeather();
	}

	void setChunkWeather(int chunkX, int chunkY, int chunkZ, weatherFlag input)
	{
		chunkPtr[{chunkX, chunkY, chunkZ}]->setWeather(input);
	}

	void chunkOverwrite(int chunkX, int chunkY, int chunkZ, chunkFlag inputChunk)
	{
		chunkPtr[{chunkX, chunkY, chunkZ}]->chunkLoad(inputChunk);
	}

	Chunk& getChunk(int chunkX, int chunkY, int chunkZ)
	{
		return *chunkPtr[{chunkX, chunkY, chunkZ}];
	}
};



