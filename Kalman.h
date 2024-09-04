// переменные для калмана
float varVolt = 2.2;  // 3.1 среднее отклонение (ищем в excel)
float varProcess = 0.15; //015 скорость реакции на изменение (подбирается вручную)
float Pc = 0.0;
float G = 0.0;
float P = 1.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe = 0.0;

float varVolt1 = 2.2;  //3.1 среднее отклонение (ищем в excel)
float varProcess1 = 0.15; //015 скорость реакции на изменение (подбирается вручную)
float Pc1 = 0.0;
float G1 = 0.0;
float P1 = 1.0;
float Xp1 = 0.0;
float Zp1 = 0.0;
float Xe1 = 0.0;

// переменные для калмана

/*
void loop() {
  int var;
  int fil_var = filter(var);
}
*/
float filterAZ(float val) {  //функция фильтрации
  Pc = P + varProcess;
  G = Pc/(Pc + varVolt);
  P = (1-G)*Pc;
  Xp = Xe;
  Zp = Xp;
  Xe = G*(val-Zp)+Xp; // "фильтрованное" значение
  return(Xe);
}

float filterUM(float val) {  //функция фильтрации
  Pc1 = P1 + varProcess1;
  G1 = Pc1/(Pc1 + varVolt1);
  P1 = (1-G1)*Pc1;
  Xp1 = Xe1;
  Zp1 = Xp1;
  Xe1 = G1*(val-Zp1)+Xp1; // "фильтрованное" значение
  return(Xe1);
}
