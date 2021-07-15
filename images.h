// serial banner
void serialbanner() {
  DBG_OUTPUT_PORT.println("");
  DBG_OUTPUT_PORT.println("");
  DBG_OUTPUT_PORT.println(" _____ ___________   _____ _     _             _ ");
  DBG_OUTPUT_PORT.println("|  ___/  ___| ___ \\ |_   _| |   (_)           | |");
  DBG_OUTPUT_PORT.println("| |__ \\ `--.| |_/ /   | | | |__  _ _ __   __ _| |");
  DBG_OUTPUT_PORT.println("|  __| `--. \\  __/    | | | '_ \\| | '_ \\ / _` | |");
  DBG_OUTPUT_PORT.println("| |___/\\__/ / |       | | | | | | | | | | (_| |_|");
  DBG_OUTPUT_PORT.println("\\____/\\____/\\_|       \\_/ |_| |_|_|_| |_|\\__, (_)");
  DBG_OUTPUT_PORT.println("                                          __/ |  ");
  DBG_OUTPUT_PORT.println("                                         |___/   ");
  DBG_OUTPUT_PORT.println("");
}

const uint8_t activeSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00011000,
    B00100100,
    B01000010,
    B01000010,
    B00100100,
    B00011000
};

const uint8_t inactiveSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00011000,
    B00011000,
    B00000000,
    B00000000
};
