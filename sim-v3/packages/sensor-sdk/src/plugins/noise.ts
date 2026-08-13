export class GaussianNoise{
  spare:number|null=null;
  next(random:()=>number):number{if(this.spare!==null){const value=this.spare;this.spare=null;return value;}const u1=Math.max(random(),Number.EPSILON),u2=random(),radius=Math.sqrt(-2*Math.log(u1)),angle=2*Math.PI*u2;this.spare=radius*Math.sin(angle);return radius*Math.cos(angle);}
}
