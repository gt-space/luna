/* @refresh reload */
import { render } from "solid-js/web";

import "../style.css";
import AVILauncher from "./AviLauncher";

render(() => <AVILauncher/>, document.getElementById("root") as HTMLElement);