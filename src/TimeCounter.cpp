#include "emcl2/TimeCounter.hpp"

using namespace std;

namespace TimeCounter
{
    TimeCounter::TimeCounter(string _path, string _node) : start(0), stop(0)
    {
        this->path = _path;
        this->node = _node;
    }

    //! 計測開始
    void TimeCounter::startCounter()
    {
        //! 開始時刻の取得
        this->start = clock();
    }

    //! 計測終了
    void TimeCounter::stopCounter()
    {
        //! 終了時刻の取得
        this->stop = clock();
        //! 差分の算出
        double ms = (double)(stop - start) / CLOCKS_PER_SEC;
        //! 処理時間 ms 変数を std::chrono::seconds::rep 型から std::string 型に変換
        std::string text = std::to_string(ms);
        this->writef(text);
    }

    void TimeCounter::printf(string msg)
    {
        cout << msg << endl;
    }

    void TimeCounter::writef(string _text)
    {
        fstream file;
        file.open(this->path, std::ios_base::app);
        if (file.is_open())
        {
            string print = "[" + this->node + "] " + _text;
            string note = _text + "\r\n";
            this->printf(print);
            file.write(note.data(), note.size());
        }
    }
}

