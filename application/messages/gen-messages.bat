protoc -obluetera_messages.pb bluetera_messages.proto
python %NRF_SDK_ROOTS%\nRF5_SDK_15.2.0_9412b96\external\nano-pb\generator\nanopb_generator.py bluetera_messages.pb