/* @refresh reload */
import { render } from "solid-js/web";

import "../../style.css";
import AHRS from "./ahrs";

render(() => <AHRS/>, document.getElementById("root") as HTMLElement);