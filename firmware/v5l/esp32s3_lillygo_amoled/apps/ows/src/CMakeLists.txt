SET(SOURCES screens/ui_MainScreen.c
    screens/ui_StrengthScreen.c
    screens/ui_RowScreen.c
    screens/ui_SystemScreen.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    fonts/ui_font_RowFont.c
    fonts/ui_font_RowFont2.c
    fonts/ui_font_WeightFont.c
    fonts/ui_font_WeightFont2.c)

add_library(ui ${SOURCES})
