set(COMPONENT_SRCS analog.c
analog-testdata.c
app_main.c
draw_osc.c
esp32_sump.c
read_chars.c
scpi-def.c
scpi_server.c
scpi_sock_server.c
sump_server.c
ota_server.c
#collector.c
sampler.c
)

#analog.c
#collector.c
set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()

