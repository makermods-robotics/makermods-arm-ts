/** Where the arm and its motors report. `console` fits. */
export interface Logger {
  info(message: string): void;
  warn(message: string): void;
  error(message: string): void;
}
