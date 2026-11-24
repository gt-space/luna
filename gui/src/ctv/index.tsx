/* @refresh reload */
import { render } from "solid-js/web";

import "../style.css";
import "./ctv.css";
import Ctv from "./Ctv";

render(() => <Ctv />, document.getElementById("root") as HTMLElement);
