#ifndef INCLUDE_GNUPLOT_REALTIME_HPP_
#define INCLUDE_GNUPLOT_REALTIME_HPP_

#include <cstdlib>
#include <gtest/gtest.h>

#include <string>
#include <sstream>

#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>

#include <exception>
#include <signal.h>
#include <vector>
#include <map>


/**
 * Gnuplotをコントロールするためのヘッダ
 *
 *
 */


#ifdef _WIN32
#ifndef GNUPLOT_PATH
  #define GNUPLOT_PATH "D:\\Tools\\GNU\\gnuplot\\bin\\pgnuplot.exe"
#endif
  #define popen(file, mode) _popen(file, mode)
  #define pclose(file) _pclose(file)
extern "C" {
  FILE *popen(char*, char*);
  void pclose(FILE*);
}
#else
#ifndef GNUPLOT_PATH
#define GNUPLOT_PATH "/usr/bin/gnuplot"
#endif
#endif


/// オプションデータ
/// @brief オプションキー, オプションの値のマップ構造
typedef std::map<const char *, const char *> plot_options_t;


/// マルチプロット用データ型
/// @details container_t : データ, オプションの組のベクタ
/// データ : データ1(時間t,xなど), データ2(f(t),yなど)の組のベクタ
typedef std::vector
               <std::pair
                       <std::vector<std::pair<double, double> >,
                       plot_options_t> > container_t;


/// 例外出力
class GnuplotException : public std::exception{
private:
  /// エラー内容
  std::string what_str;
  public:
  /// コンストラクタ(gnplot_realtimeエラー出力)
  /// @param what_arg エラー内容
  GnuplotException(const std::string &what_arg) : what_str(what_arg){}
  /// デストラクタ
  ~GnuplotException() throw(){}
  /// エラー内容出力オーバーライド関数
  const char *what() const throw(){
    return what_str.c_str();
  }
};


/// プロットクラス
class Gnuplot{
private:
  /// プロット用ファイル変数
  FILE* fp_;
public:
  /// コンストラクタ
  /// @exception gnuplotを開けない場合エラー
  Gnuplot()
  throw(GnuplotException) {
    this->set_signal(SIGINT);
    fp_ = popen(GNUPLOT_PATH, "w");
    if (fp_ == NULL) {
      throw GnuplotException(std::string("can't find gnuplot."));
    }
  }

  /// デストラクタ
  ~Gnuplot(){
    if (fp_ != NULL) {
      pclose(fp_);
      // delete fp_;
    }
  }

  /// 割り込み終了の設定
  void set_signal(int p_signame) {
    if (signal(p_signame, this->sig_handler) == SIG_ERR) {
      // シグナル設定エラー
      std::runtime_error("シグナルの設定が出来ませんでした。終了します\n");
      exit(1);
    }
    return;
  }

  /// シグナル受信/処理
  static void sig_handler(int p_signame)
  {
    std::cout << "割り込みです。終了します" << std::endl;
    // if (fp_ != NULL) {
      // fprintf(fp_, "q\n");
      // fflush(fp_);
    //   pclose(fp_);
    // }
    exit(0);

    return;
  }

  /// グラフ出力(string)
  /// @param stream 自身Gnuplotオブジェクト
  /// @param command gnuplotコマンド内容(文字列)
  friend Gnuplot &operator << (Gnuplot &stream, const std::string &command) {
    fprintf(stream.fp_, "%s", command.c_str());
    return stream;
  }

  // グラフ出力(int)
  /// @param stream 自身Gnuplotオブジェクト
  /// @param command gnuplotコマンド内容(整数値)
  /// @return 自身Gnuplotオブジェクト
  friend Gnuplot &operator<<(Gnuplot &stream, const int &value){
    fprintf(stream.fp_, "%d", value);
    return stream;
  }

  // グラフ出力(double)
  /// @param stream 自身Gnuplotオブジェクト
  /// @param command gnuplotコマンド内容(実数値)
  /// @return 自身Gnuplotオブジェクト
  friend Gnuplot &operator<<(Gnuplot &stream, const double &value){
    fprintf(stream.fp_, "%f", value);
    return stream;
  }

  /// フラッシュする
  void flush() {
    fflush(fp_);
  }
};



///
class GnuplotServer {
protected:
  ///
  Gnuplot gp_;
  ///
  bool mulitplot_frag_;
public:
  /// コンストラクタ
  GnuplotServer();
  /// デストラクタ
  ~GnuplotServer();

  ///
  GnuplotServer& set_multiplot();

  ///
  GnuplotServer& set(const std::string& param);

  ///
  void flush();

  ///
  void pause(const int& time);

  ///
  template<class Fanctor>
  GnuplotServer& plot(const Fanctor& plotter);

  ///
  GnuplotServer& plot(const std::vector<std::pair<double, double> > &data,
                      const plot_options_t &options);

  ///
  GnuplotServer& plot(const std::vector<std::pair<double, double> > &data);

  ///
  GnuplotServer& plot(const container_t &data_list);

};



#endif // INCLUDE_GNUPLOT_REALTIME_HPP_
