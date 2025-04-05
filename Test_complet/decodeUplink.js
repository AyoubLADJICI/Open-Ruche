 function decodeUplink(input) {
 var data = {};
 data.key = "sqwvhtcqsnlaq27y";
 data.t = (input.bytes[1] << 8 | (input.bytes[0])) / 100; //temp_sht31
 data.h = (input.bytes[3] << 8 | (input.bytes[2])) / 100; //hum_sht31
 data.t_0 = (input.bytes[5] << 8 | (input.bytes[4])) / 100; //temp_ds18b20_1
 data.t_1 = (input.bytes[7] << 8 | (input.bytes[6])) / 100; //temp_ds18b20_2
 data.t_2 = (input.bytes[9] << 8 | input.bytes[8]) / 100;  // temp_dht22
 data.t_3 = (input.bytes[11] << 8 | input.bytes[10]) / 100; //hum_dht22 Probleme avec la variable h_i donc on utilise a la place t_3
 data.weight_kg = (input.bytes[13] << 8 | (input.bytes[12])) / 100; //poids
 data.l = (input.bytes[15] << 8 | (input.bytes[14])); //lux
 data.bv = (input.bytes[17] << 8 | (input.bytes[16])) / 100; //batterie
 data.s_fan_4 = (input.bytes[19] << 8 | (input.bytes[18])) / 100; //presence de la reine
 return {
   data: data,
   warnings: [],
   errors: []
  };
 }