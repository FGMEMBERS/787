var Radio = gui.Dialog.new("/sim/gui/dialogs/radios/dialog",
        "Aircraft/787/Systems/radio.xml");

gui.menuBind("radio", "dialogs.Radio.open()");
