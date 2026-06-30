from fibre.testing import test_runner


# Legacy incremental, Hall, sin/cos, and concrete SPI absolute encoder tests were
# removed with the corresponding firmware modes. MT6826S production validation is
# covered by Firmware/Tests/hex_4342_mt6826s.
tests = []


if __name__ == '__main__':
    test_runner.run(tests)
