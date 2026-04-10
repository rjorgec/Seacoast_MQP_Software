idf_component_register(
  SRCS "pico_link.c"
       "../../../shared/proto/proto_crc.c"
       "../../../shared/proto/cobs.c"
  INCLUDE_DIRS "include"
               "../../../shared/proto"
  REQUIRES driver freertos
)
