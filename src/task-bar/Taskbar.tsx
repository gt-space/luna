import { createSignal } from "solid-js";
import yjsplogo from "./assets/2020+Logo+White.png";
import { invoke } from "@tauri-apps/api/tauri";
import "../App.css";
import { SimpleTitleBar } from "../window-components/TitleBar";
import {Router} from 'solid-app-router';
import MenuBar from "./MenuBar";
import Body from "./Body";
import Footer from "../window-components/Footer";

function Taskbar() {
  return <div class="taskbar">
    <div>
      <SimpleTitleBar/>
    </div>
    <div>
      <MenuBar/>
    </div>
    <Body/>
    <Footer/>
  </div>
}

export default Taskbar;
