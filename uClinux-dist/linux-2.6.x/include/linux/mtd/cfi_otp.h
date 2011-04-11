/* Lab X CFI OTP Support */

typedef enum otp_addr {
  REGISTER1,
  REGISTER2,
  REGISTER3,
  REGISTER4,
  REGISTER5,
  REGISTER6,
  REGISTER7,
  REGISTER8,
  REGISTER9,
  REGISTER10,
  REGISTER11,
  REGISTER12,
  REGISTER13,
  REGISTER14,
  REGISTER15,
  REGISTER16
} otp_register;

typedef uint8_t securityword_t[16];

int read_otp_reg(otp_register addr, securityword_t *otp);
