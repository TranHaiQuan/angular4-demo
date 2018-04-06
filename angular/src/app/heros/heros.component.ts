import { Component, OnInit } from '@angular/core';
// import { HEROES } from '../mock-heros';
import { HeroService } from '../hero.service'
import { Hero } from './hero';

@Component({
  selector: 'app-heros',
  templateUrl: './heros.component.html',
  styleUrls: ['./heros.component.css']
})
export class HerosComponent implements OnInit {
	heroes: Hero[];

  constructor(private heroService: HeroService) { }

  getHeroes(): void {
    // this.heroes = this.heroService.getHeros();
    this.heroService.getHeros()
      .subscribe(heroes => this.heroes = heroes);

  }

  ngOnInit() {
    this.getHeroes();
  }

}
