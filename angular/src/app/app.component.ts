import { Component } from '@angular/core';
import { Http } from '@angular/http'

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.css']
})
export class AppComponent {
	// users = [];
	// contructor(private http:Http) {
	// 	this.getUsers();
	// }

	// getUsers(){
	// 	this.http.get('http://localhost:3000/users')
	// 		.subscribe(res => {
	// 			this.users = res.json().users;
	// 		});
	// }
  title = 'Angular Demo';
}
