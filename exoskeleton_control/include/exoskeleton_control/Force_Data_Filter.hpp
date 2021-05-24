#include <queue>

float * Force_Filter(int Force[4])
{
    static float Force[4];

    return Force;
}
class Data_Filter
{
public:
    Data_Filter(int Num){N = Num};
    ~Data_Filter(){};
    float Filtering(float Data);

private:
    int N;
    queue<float> Data_Set;
};

float Filtering(float Data)
{
    if(Data_Set.size() < N)
    {
        Data_Set.push(Data);
        float Data_Array[N];
        float Average_Data = 0;
        float Size = Data_Set.size();

        for (int i = 0; i < Size; i++)
        {
            Data_Array[i] = Data_Set.front();
            Data_Set.pop();
            Average_Data += Data_Array[i];
        }
        for (int i = 0; i < Size; i++)
        {
            Data_Set.push(Data_Array[i]);
        }
        Average_Data = Average_Data/Size;
        return Average_Data;
    }
    else
    {
        for (int i = 0; i < Size; i++)
        {
            Data_Set.pop();
            Data_Array[i] = Data_Set.front();
            Data_Set.pop();
            Average_Data += Data_Array[i];
        }
        for (int i = 0; i < Size; i++)
        {
            Data_Set.push(Data_Array[i]);
        }
        Average_Data = Average_Data/Size;
        return Average_Data;
    }
}