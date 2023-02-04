import { createSignal } from "solid-js";
import yjsplogo from "./assets/2020+Logo+White.png";
import { invoke } from "@tauri-apps/api/tauri";
import "../App.css";
import SimpleTitleBar from "../window-components/TitleBar";
import {Router} from 'solid-app-router';
import MenuBar from "./MenuBar";

function Taskbar() {
  return <div>
    <SimpleTitleBar/>
    <MenuBar/>
  </div>
}

export default Taskbar;
